/**
 * @file main.cpp
 * @author Tanner Broaddus (jbroaddus@path-robotics.com)
 * @brief
 *
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <vector>

// Internal headers
#include <thread_safe_event.h>

// Third party includes
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

/**
 * @brief Macro for const string values; aka "Definition"
 *
 */
#define DEFINITION_TYPE static constexpr const char* const

/**
 * @brief Function-like macro for printing a message and exiting the program with a failure status
 *
 */
#define EXIT_FAILURE_WITH_MESSAGE(message) \
  std::cerr << message << std::endl;       \
  std::exit(EXIT_FAILURE)

// Namespace aliases

namespace po_ns = boost::program_options;

// Type aliases

using TimePoint_t = std::chrono::system_clock::time_point;  // time_point type alias
using Sequence_t = std::vector<YAML::Node>;                 // Yaml sequence type
using OrderQuantityMap_t = std::map<int, int>;

// Definitions

DEFINITION_TYPE HELP_OPT_STR{ "help,h" };                     // help option values
DEFINITION_TYPE HELP_OPT_KEY_STR{ "help" };                   // help option key
DEFINITION_TYPE FREQUENCY_OPT_STR{ "frequency,f" };           // frequency option values
DEFINITION_TYPE FREQUENCY_OPT_KEY_STR{ "frequency" };         // frequency option key
DEFINITION_TYPE ODIR_OPT_STR{ "odir,o" };                     // output directory option values
DEFINITION_TYPE ODIR_OPT_KEY_STR{ "odir" };                   // output directory option key
DEFINITION_TYPE IPATH_EVENT_LIST_OPT_STR{ "elistpath,e" };    // event list input path option values
DEFINITION_TYPE IPATH_EVENT_LIST_OPT_KEY_STR{ "elistpath" };  // event list input path option key
DEFINITION_TYPE DESC_MSG_STR{
  "Records memory order availability for the buddy allocator over the life cycle of the program.\nAllowed "  // Description
                                                                                                             // message
  "options"
};
DEFINITION_TYPE DEFAULT_OUTPUT_DIR_PATH_STR{ "/var/local/buddy-recorder/sessions" };  // Default output directory path
DEFINITION_TYPE BUDDYINFO_PATH_STR{ "/proc/buddyinfo" };                              // buddyinfo file path
DEFINITION_TYPE OUTPUT_FILE_PREFIX{ "buddy-recorder_" };                              // buddy recorder output file prefix

// Other static variables

static buddy_recorder::ThreadSafeEvent CURRENT_EVENT{ "INIT" };     // Thread-safe global variable for current event (starts with "INIT")
static std::exception_ptr P_RECORDING_THREAD_EXCEPTION{ nullptr };  // Exception pointer to propagate exception thrown in the child recording thread
                                                                    // to parent thread
static constexpr int DEFAULT_FREQUENCY_HZ{ 1 };                     // Default pause time (if no value is given via the CLI option)
static constexpr int MAIN_POLLING_LOOP_WAIT_TIME_MS{ 50 };          // Main polling loop wait time
static constexpr char EXIT_CHARACTER{ 'x' };                        // Input character for exiting the program
static constexpr char LOG_EVENT_CHARACTER{ 'l' };                   // Input chracter for logging an event
static termios OLD_T;
static termios NEW_T;

// Static functions

/**
 * @brief Get the Normal Line object
 *
 * @return std::string
 */
[[nodiscard]] static inline std::pair<std::string, TimePoint_t> getNormalLineWithTimePoint()
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
[[nodiscard]] static OrderQuantityMap_t getOrderQuantityMap(const std::string normal_line_str)
{
  OrderQuantityMap_t order_quantity_map;

  auto pos = normal_line_str.find("Normal");
  if (pos == std::string::npos)
  {
    throw std::runtime_error{ "Normal line str does not contain \"Normal\" sub str t  std::exit(signum);o parse with!" };
  }

  std::stringstream ss(normal_line_str.substr(pos + 6));  // Incrementing the starting position of the sub str by a constant 6 characters as that is
                                                          // the number of characters in "Normal"
  int order { 0 };
  int val;
  while (ss >> val)  // exctracting out each value for every order reported
  {
    order_quantity_map.emplace(order, val);
    ++order;
  }

  return order_quantity_map;
}

/**
 * @brief
 *
 * @param r_time_point
 * @return std::string
 */
[[nodiscard]] static inline std::string formatTimePoint(const TimePoint_t& r_time_point)
{
  auto time = std::chrono::system_clock::to_time_t(r_time_point);
  std::tm tm = *std::localtime(&time);

  std::stringstream ss;
  ss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");

  return ss.str();
}

/**
 * @brief
 *
 * @param r_time_point
 * @return std::string
 */
[[nodiscard]] static inline std::string formatTimePointHumanReadable(const TimePoint_t& r_time_point)
{
  auto time = std::chrono::system_clock::to_time_t(r_time_point);
  std::tm tm = *std::localtime(&time);

  std::stringstream ss;
  ss << std::put_time(&tm, "%B %d, %Y %H:%M:%S");

  return ss.str();
}

/**
 * @brief
 *
 * @param r_time_point
 * @return std::string
 */
[[nodiscard]] static inline std::string generateFileName(const TimePoint_t& r_time_point)
{
  return OUTPUT_FILE_PREFIX + formatTimePoint(r_time_point) + ".yaml";
}

/**
 * @brief
 *
 * @param r_node
 * @param r_directory_path
 * @param r_file_name
 */
static inline void saveYamlNodeToFile(const YAML::Node& r_node, const std::string& r_directory_path, const std::string& r_file_name)
{
  std::filesystem::path output_file_path{ r_directory_path + r_file_name };

  std::cout << "Saving session information to " << output_file_path << std::endl;

  if (!std::filesystem::exists(r_directory_path))
  {
    std::filesystem::create_directories(r_directory_path);
  }

  std::ofstream output_file{ output_file_path };

  if (!output_file.is_open())
  {
    throw std::runtime_error{ "Could not open output file " + output_file_path.string() };
  }

  output_file << r_node;

  output_file.close();
}

/**
 * @brief Checks the availability of input from the terminal
 *
 * @return true IF input is available
 * @return false IF input is NOT available
 */
[[nodiscard]] static inline bool isInputAvailable() noexcept
{
  timeval tv;
  fd_set readfds;

  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  return select(1, &readfds, NULL, NULL, &tv) > 0;
}

/**
 * @brief
 *
 */
static inline void applyTerminalSettings()
{
  tcgetattr(STDIN_FILENO, &OLD_T);
  NEW_T = OLD_T;
  NEW_T.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &NEW_T);
}

/**
 * @brief
 *
 */
static inline void cleanupTerminalSettings()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &OLD_T);
}

/**
 * @brief
 *
 * @param signum
 */
static inline void cleanupTerminalSettingsAfterSignal(int signum)
{
  cleanupTerminalSettings();

  if (signum == SIGINT)
  {
    std::cerr << "Program was interrupted! Exiting..." << std::endl;
  }
  if (signum == SIGTERM)
  {
    std::cerr << "Program was terminated! Exiting..." << std::endl;
  }

  std::exit(EXIT_FAILURE);
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
  YAML::Node node;               // YAML node to store the recorded sequence into]
  std::string current_event;

  try
  {
    while (true)
    {
      if (parent_signal_stop == true)
      {
#ifdef DEBUG_LOGS
        std::cout << "Parent has signalled to stop the child, breaking the main child loop" << std::endl;
#endif  // DEBUG_LOGS
        break;
      }
      // Get normal line from buddyinfo file with time point

      auto [line, time_point] = getNormalLineWithTimePoint();

      // Extract the order values from the normal line

      const auto order_quantity_map = getOrderQuantityMap(line);

      // Add entry to recorded sequence!

      YAML::Node entry_node;
      entry_node["order_quantities"] = order_quantity_map;
      entry_node["formatted_time"] = formatTimePointHumanReadable(time_point);
      entry_node["event"] = CURRENT_EVENT.getEvent();
      entry_node["time_stamp"] = formatTimePoint(time_point);

      recorded_sequence.push_back(std::move(entry_node));

      std::this_thread::sleep_for(std::chrono::milliseconds(pause_time_ms));
    }
  }
  catch (const std::exception& r_exception)
  {
    P_RECORDING_THREAD_EXCEPTION = std::current_exception();
    child_signal_abort = true;
  }
  catch (...)
  {
    std::cerr << "Unknown object was thrown and caught in the child thread" << std::endl;
    P_RECORDING_THREAD_EXCEPTION = nullptr;
    child_signal_abort = true;
  }

  node["sequence"] = recorded_sequence;

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

  applyTerminalSettings();
  signal(SIGINT, cleanupTerminalSettingsAfterSignal);
  signal(SIGTERM, cleanupTerminalSettingsAfterSignal);
  atexit(cleanupTerminalSettings);

  // Define options

  po_ns::options_description desc(DESC_MSG_STR);
  desc.add_options()(HELP_OPT_STR, "Display help message")(
      FREQUENCY_OPT_STR, po_ns::value<float>()->default_value(DEFAULT_FREQUENCY_HZ), "Frequency of checking the memory order in hertz")(
      ODIR_OPT_STR, po_ns::value<std::string>()->default_value(DEFAULT_OUTPUT_DIR_PATH_STR), "Output directory path")(
      IPATH_EVENT_LIST_OPT_STR, po_ns::value<std::string>(), "Input path to event list file");

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

  const auto frequency_hz = vm[FREQUENCY_OPT_KEY_STR].as<float>();
  auto output_directory_path_str = vm[ODIR_OPT_KEY_STR].as<std::string>();

  std::string event_list_input_path;
  if (vm.count(IPATH_EVENT_LIST_OPT_KEY_STR))
  {
    event_list_input_path = vm[IPATH_EVENT_LIST_OPT_KEY_STR].as<std::string>();
  }

  // Append a trailing forward slash to output directory (if necessary)
  if (output_directory_path_str.back() != '/')
  {
    output_directory_path_str.append("/");
  }

  // Print option values

  std::cout << "=== Parameters ===" << '\n';
  std::cout << "Frequency (Hz): " << frequency_hz << '\n';
  std::cout << "Output directory: " << output_directory_path_str << '\n';
  std::cout << "Event list input path: " << (event_list_input_path.empty() ? "N/A" : event_list_input_path) << std::endl;

  std::cout << "\n=== Usage ===" << '\n';
  std::cout << "x - exit program and generate a session file" << '\n';
  std::cout << "l - log next event from the event ready queue" << std::endl;

  // Parse event list input file (if path has been provided)

  std::deque<std::string> event_ready_queue;

  if (!event_list_input_path.empty())
  {
    if (!std::filesystem::exists(event_list_input_path))
    {
      EXIT_FAILURE_WITH_MESSAGE("Event list path does not refer to an existing file: '" << event_list_input_path);
    }

    std::ifstream event_list_input_file{ event_list_input_path };
    if (!event_list_input_file.is_open())
    {
      EXIT_FAILURE_WITH_MESSAGE("Problem with opening event list input file (" << event_list_input_path << ")");
    }

    std::string line;

    while (std::getline(event_list_input_file, line))
    {
      event_ready_queue.push_back(std::move(line));
    }

    event_list_input_file.close();

    if (event_ready_queue.empty())
    {
      std::cout << "Warning: Event list input file parsed but event ready queue is empty. Likely an empty file that was passed in." << std::endl;
    }
  }

  // Calculate pause time from targeted frequency value

  const auto pause_time_ms = (1.f / frequency_hz) * 1000;

  // Main try-catch block
  try
  {
    // Launch the recording thread

    std::atomic_bool parent_signal_stop{ false };  // atomic boolean to represent the parent signal to the child thread to stop
    std::atomic_bool child_signal_abort{ false };  // atomic boolean to represent the child signal to the parent thread that it has aborted (an
                                                   // internal exception has been thrown)
    auto future =
        std::async(std::launch::async, recordingThreadEntryPoint, std::ref(parent_signal_stop), std::ref(child_signal_abort), pause_time_ms);

    std::cout << "\n=== Recording started ===" << std::endl;
    std::cout << "Next event in the event ready queue: \"" << event_ready_queue.front() << '\"' << std::endl;

    // Main driver loop
    while (!child_signal_abort)
    {
      // Evaluate input if input is available via the terminal
      if (isInputAvailable())
      {
        const char input = getchar();

        if (input == EXIT_CHARACTER)
        {
#ifdef DEBUG_LOGS
          std::cout << "User has pressed the exit character! Setting parent_signal_stop=true and breaking the main polling loop" << std::endl;
#endif  // DEBUG_LOGS
          parent_signal_stop = true;
          break;
        }
        else if (input == LOG_EVENT_CHARACTER)
        {
          if (!event_ready_queue.empty())
          {
            CURRENT_EVENT = std::move(event_ready_queue.front());
            event_ready_queue.pop_front();

            if (!event_ready_queue.empty())
            {
              std::cout << '"' << CURRENT_EVENT.getEvent() << "\" event logged! Next event in the queue: \"" << event_ready_queue.front() << '"'
                        << std::endl;
            }
            else
            {
              std::cout << '"' << CURRENT_EVENT.getEvent() << "\" event logged; event ready queue is empty!" << std::endl;
            }
          }
          else
          {
            std::cout << "WARNING: Attempted to log event with an empty event queue" << std::endl;
          }
        }
        else
        {
          std::cout << "WARNING: Input is not valid!" << std::endl;
        }
      }

      if (child_signal_abort)
      {
        std::cout << "Child thread signalled to abort, exiting main polling loop in parent thread" << std::endl;
        break;
      }

      // Sleep for a period of time
      std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_POLLING_LOOP_WAIT_TIME_MS));
    }

    future.wait();

    // Check if child thread signalled to abort
    if (child_signal_abort)
    {
      // Check if there is an active exception stored in the exception ptr global variable populated by the child thread to rethrow in the current
      // thread
      if (P_RECORDING_THREAD_EXCEPTION != nullptr)
      {
        // Rethrow exception raised in child thread
        std::rethrow_exception(P_RECORDING_THREAD_EXCEPTION);
      }

      // No exception raised in child thread, throw exception in parent thread to indicate that the child thread aborted without an exception thrown
      throw std::runtime_error{ "No exception from child thread to rethrow even though child thread aborted!" };
    }

    auto node = future.get();

    saveYamlNodeToFile(node, output_directory_path_str + formatTimePoint(start_time) + '/', generateFileName(start_time));
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
