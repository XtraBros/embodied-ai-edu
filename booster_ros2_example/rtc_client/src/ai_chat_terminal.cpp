#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster/robot/ai/api.hpp"
#include "include/prompt.h"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

static booster_interface::msg::BoosterApiReqMsg MakeMsg(const int64_t api,
                                                        const std::string& body) {
  booster_interface::msg::BoosterApiReqMsg msg;
  msg.api_id = api;
  msg.body = body;
  return msg;
}

static bool CallApi(const rclcpp::Node::SharedPtr& node,
                    const rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr& client,
                    int64_t api_id,
                    const std::string& body,
                    std::chrono::milliseconds timeout = 5s) {
  auto req = std::make_shared<booster_interface::srv::RpcService::Request>();
  req->msg = MakeMsg(api_id, body);
  auto future = client->async_send_request(req);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "RpcService call timed out (api_id=%ld)", api_id);
    return false;
  }
  auto response = future.get()->msg;
  if (response.status != 0) {
    RCLCPP_WARN(node->get_logger(), "RpcService returned status=%ld", response.status);
  }
  return true;
}

static int64_t NowMillis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

static std::string ShellEscape(const std::string& input) {
  std::string escaped = "'";
  for (char c : input) {
    if (c == '\'') {
      escaped += "'\\''";
    } else {
      escaped += c;
    }
  }
  escaped += "'";
  return escaped;
}

static std::string RunCommand(const std::string& command) {
  std::array<char, 256> buffer{};
  std::string result;
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return result;
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);
  if (!result.empty() && result.back() == '\n') {
    result.pop_back();
  }
  return result;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ai_chat_terminal");
  auto rtc_client =
      node->create_client<booster_interface::srv::RpcService>("booster_rtc_service");
  const auto llm_cmd = node->declare_parameter<std::string>("llm_cmd", "");

  while (!rtc_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "booster_rtc_service not available, waiting...");
  }

  bool tts_only = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--tts-only") {
      tts_only = true;
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  std::string body;
  if (!tts_only) {
    booster::robot::StartAiChatParameter param;
    param.llm_config_.system_prompt_ = kDefaultSystemPrompt;
    param.llm_config_.welcome_msg_ = kWelcomeMsg;
    param.interrupt_mode_ = false;
    param.asr_config_.interrupt_speech_duration_ = 200;
    param.tts_config_.ignore_bracket_text_ = {3};
    param.tts_config_.voice_type_ = "zh_female_shuangkuaisisi_emo_v2_mars_bigtts";
    param.enable_face_tracking_ = false;
    body = param.ToJson().dump();

    if (!CallApi(node, rtc_client, static_cast<int64_t>(booster::robot::AiApiId::kStartAiChat),
                 body)) {
      executor.cancel();
      spin_thread.join();
      return 1;
    }
  }

  if (tts_only) {
    std::cout << "TTS-only mode. Type text to speak, or /quit to exit." << std::endl;
  } else if (llm_cmd.empty()) {
    std::cout << "AI chat started. Type text to speak, or /quit to exit." << std::endl;
  } else {
    std::cout << "AI chat started. Type text to get an AI response, or /quit to exit."
              << std::endl;
  }

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    if (line == "/quit") {
      break;
    }
    std::string response = line;
    if (!llm_cmd.empty()) {
      std::string command = llm_cmd;
      const std::string placeholder = "{input}";
      const auto escaped = ShellEscape(line);
      size_t pos = command.find(placeholder);
      if (pos != std::string::npos) {
        while (pos != std::string::npos) {
          command.replace(pos, placeholder.size(), escaped);
          pos = command.find(placeholder, pos + escaped.size());
        }
      } else {
        command += " " + escaped;
      }
      response = RunCommand(command);
      if (response.empty()) {
        response = line;
      }
      std::cout << "AI: " << response << std::endl;
    }
    booster::robot::SpeakParameter speak_param;
    speak_param.msg_ = response;
    body = speak_param.ToJson().dump();
    CallApi(node, rtc_client, static_cast<int64_t>(booster::robot::AiApiId::kSpeak), body);
  }

  if (!tts_only) {
    CallApi(node, rtc_client, static_cast<int64_t>(booster::robot::AiApiId::kStopAiChat), "");
  }
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
