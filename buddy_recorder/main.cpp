/**
 * @file main.cpp
 * @author Tanner Broaddus (jbroaddus@path-robotics.com)
 * @brief
 *
 */
#include <atomic>
#include <exception>
#include <iostream>
#include <regex>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <filesystem>
#include <future>
#include <termios.h>

// TODO get termios functionality up and going for non-blocking key press check!
// Look into this solution --> https://stackoverflow.com/questions/27968446/test-whether-key-is-pressed-without-blocking-on-linux

// Third party includes
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

// Macro for static const str type (assigned at compile time); aka "Definition"
#define DEFINITION_TYPE static constexpr const char* const

// Namespace aliases

namespace po_ns = boost::program_options;

// Type aliases

using TimePoint_t = std::chrono::system_clock::time_point;               // time_point type alias
using SequenceEntry_t = std::tuple<std::vector<int>, std::string, int>;  // Yaml sequence entry type
using Sequence_t = std::vector<SequenceEntry_t>;                         // Yaml sequence type

// Definitions

DEFINITION_TYPE HELP_OPT_STR{ "help,h" };                // help option values
DEFINITION_TYPE HELP_OPT_KEY_STR{ "help" };              // help option key
DEFINITION_TYPE PAUSE_TIME_OPT_STR{ "pause_time,p" };    // pause time option values
DEFINITION_TYPE PAUSE_TIME_OPT_KEY_STR{ "pause_time" };  // pause time option key
DEFINITION_TYPE ODIR_OPT_STR{ "odir,o" };                // output directory option values
DEFINITION_TYPE ODIR_OPT_KEY_STR{ "odir" };              // output directory option keys
DEFINITION_TYPE DESC_MSG_STR{
  "Records memory order availability for the buddy allocator over the life cycle of the program.\nAllowed "  // Description
                                                                                                             // message
  "options"
};
DEFINITION_TYPE DEFAULT_OUTPUT_DIR_PATH_STR{ "/var/local/buddy-recorder/" };  // Default output directory path
DEFINITION_TYPE BUDDYINFO_PATH_STR{ "/proc/buddyinfo" };                      // buddyinfo file path
DEFINITION_TYPE OUTPUT_FILE_PREFIX{ "buddy-recorder_" };                      // buddy recorder output file prefix

// Other static variables

static constexpr int DEFAULT_PAUSE_TIME_MS{ 0 };                    // Default pause time (if no value is given via the CLI option)
static std::exception_ptr P_RECORDING_THREAD_EXCEPTION{ nullptr };  // Exception pointer to propagate exception thrown in the child recording thread
                                                                    // to parent thread

// Static functions

/**
 * @brief Get the Normal Line object
 *
 * @return std::string
 */
[[nodiscard]] static std::pair<std::string, TimePoint_t> getNormalLineWithTimePoint()
{
  std::ifstream input_file(BUDDYINFO_PATH_STR);

  if (!input_file.is_open())
  {
    throw std::runtime_error{ "Could not open buddyinfo file!" };
  }

  const auto now = std::chrono::system_clock::now();

  static std::string line;

  while (std::getline(input_file, line))
  {
    if (line.find("Normal") != std::string::npos)
    {
      return { line, now };
    }
  }

  throw std::runtime_error{ "Could not find \"Normal\" region of memory in buddyinfo file!" };
}

/**
 * @brief Get the Order Values object
 *
 * @param normal_line_str
 * @return std::vector<int>
 */
[[nodiscard]] static std::vector<int> getOrderValues(const std::string normal_line_str)
{
  std::vector<int> order_values_vec;

  auto pos = normal_line_str.find("Normal");
  if (pos == std::string::npos)
  {
    throw std::runtime_error{ "Normal line str does not contain \"Normal\" sub str to parse with!" };
  }

  std::stringstream ss(normal_line_str.substr(pos + 6));  // Incrementing the starting position of the sub str by a constant 6 characters as that is
                                                          // the number of characters in "Normal"

  int val;
  while (ss >> val)  // exctracting out each value for every order reported
  {
    order_values_vec.push_back(val);
  }

  return order_values_vec;
}

/**
 * @brief
 *
 * @param r_time_point
 * @return std::string
 */
[[nodiscard]] static inline std::string generateFileName(const TimePoint_t& r_time_point)
{
  auto time = std::chrono::system_clock::to_time_t(r_time_point);
  std::tm tm = *std::localtime(&time);

  std::stringstream ss;

  ss << OUTPUT_FILE_PREFIX << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".yaml";

  return ss.str();
}

/**
 * @brief
 *
 * @param r_node
 * @param r_file_path_str
 */
[[nodiscard]] static inline void saveYamlNode(const YAML::Node& r_node, const std::string& r_directory_path, const std::string& r_file_name)
{
}

/**
 * @brief
 *
 * @param parent_signal_stop
 * @param child_signal_abort
 * @param pause_time_ms
 * @return YAML::Node
 */
[[nodiscard]] YAML::Node recordingThreadEntryPoint(std::atomic_bool& parent_signal_stop,
                                                   std::atomic_bool& child_signal_abort,
                                                   const int pause_time_ms)
{
  Sequence_t recorded_sequence;  // Recorded sequence to store into YAML node;
  YAML::Node node;               // YAML node to store the recorded sequence into

  try
  {
    while (true)
    {
      if (parent_signal_stop == true)
      {
        exit;
      }
      // Get normal line from buddyinfo file with time point

      auto [line, time_point] = getNormalLineWithTimePoint();

      // Extract the order values from the normal line

      const auto order_values_vec = getOrderValues(line);

      // TODO add entry to recorded sequence!

      std::this_thread::sleep_for(std::chrono::milliseconds(pause_time_ms));
    }
  }
  catch (std::exception& r_exception)
  {
    P_RECORDING_THREAD_EXCEPTION = std::current_exception();
    child_signal_abort = true;
  }
  catch (...)
  {
    P_RECORDING_THREAD_EXCEPTION = nullptr;
    child_signal_abort = true;
  }

  return node;
}

/**
 * @brief Main entry point
 *
 */
int main(int argc, char** argv)
{
  // Acquire start time of the program
  auto start_time = std::chrono::system_clock::now();

  // Define options

  po_ns::options_description desc(DESC_MSG_STR);
  desc.add_options()(HELP_OPT_STR, "Display help message")(
      PAUSE_TIME_OPT_STR, po_ns::value<int>()->default_value(DEFAULT_PAUSE_TIME_MS), "Pause time (in milliseconds) for every iteration")(
      ODIR_OPT_STR, po_ns::value<std::string>()->default_value(DEFAULT_OUTPUT_DIR_PATH_STR), "Output directory path");

  // Parse the CLI options and store valid entries in a Boost variable map

  po_ns::variables_map vm;
  po_ns::store(po_ns::parse_command_line(argc, argv, desc), vm);
  po_ns::notify(vm);

  // Check if the help option was passed

  if (vm.count(HELP_OPT_KEY_STR))
  {
    std::cout << desc << std::endl;  // Print help message
    return EXIT_SUCCESS;             // Return SUCCESS status
  }

  // Extract option values

  const auto pause_time_ms = vm[PAUSE_TIME_OPT_KEY_STR].as<int>();
  auto output_directory_path_str = vm[ODIR_OPT_KEY_STR].as<std::string>();

  // Append a trailing forward slash if necessary
  if (output_directory_path_str.back() != '/')
  {
    output_directory_path_str.append("/");
  }

  // Print option values

  std::cout << "=== Parameters ===" << '\n';
  std::cout << "Pause time (ms): " << pause_time_ms << '\n';
  std::cout << "Output directory: " << output_directory_path_str << std::endl;

  // Main try-catch block
  try
  {
    // Launch the recording thread

    std::atomic_bool parent_signal_stop{ false };  // atomic boolean to represent the parent signal to the child thread to stop
    std::atomic_bool child_signal_abort{ false };  // atomic boolean to represent the child signal to the parent thread that it has aborted (an
                                                   // internal exception has been thrown)
    auto future =
        std::async(std::launch::async, recordingThreadEntryPoint, std::ref(parent_signal_stop), std::ref(child_signal_abort), pause_time_ms);

    // Main driver loop
    while (true)
    {
      // Wait for user feedback for the child thread to be signalled to stop
    }

    future.wait();

    auto node = future.get();

    saveYamlNode(node, output_directory_path_str, generateFileName(start_time));
  }
  catch (const std::exception& r_exception)
  {
    std::cerr << "Exception thrown while running - " << r_exception.what() << "\nExiting program." << std::endl;
    return EXIT_FAILURE;  // Return FAILURE status
  }
  catch (...)
  {
    std::cerr << "Unrecognized object thrown while running, exiting program." << std::endl;
    return EXIT_FAILURE;  // Return FAILURE status
  }

  return EXIT_SUCCESS;  // Return SUCCESS status
}
