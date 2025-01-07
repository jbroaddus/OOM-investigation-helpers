/**
 * @file thread_safe_queue.h
 * @author your name (you@domain.com)
 * @brief
 *
 *
 */
#ifndef BUDDY_RECORDER_THREAD_SAFE_EVENT_H
#define BUDDY_RECORDER_THREAD_SAFE_EVENT_H

#include <string>
#include <mutex>

namespace buddy_recorder
{
/**
 * @brief Non-copyable/moveable threadsafe event
 *
 */
class ThreadSafeEvent
{
public:
  // Default constructor / destructor

  ThreadSafeEvent() = default;
  ~ThreadSafeEvent() = default;

  // Deleted copy operations

  ThreadSafeEvent(const ThreadSafeEvent&) = delete;

  ThreadSafeEvent& operator=(const ThreadSafeEvent&) = delete;

  // Deleted move operations

  ThreadSafeEvent(ThreadSafeEvent&&) = delete;
  ThreadSafeEvent& operator=(ThreadSafeEvent&&) = delete;

  // Public member functions

  /**
   * @brief Construct a new Thread Safe Event object
   *
   * @param event_str
   */
  ThreadSafeEvent(std::string event_str) : event_str_{ std::move(event_str) }
  {
  }

  /**
   * @brief
   *
   * @param event
   * @return ThreadSafeEvent&
   */
  ThreadSafeEvent& operator=(std::string event);

  /**
   * @brief Get the Event object
   *
   * @return std::string
   */
  [[nodiscard]] std::string getEvent();

private:
  // Private member variables

  std::string event_str_;
  std::mutex mutex_;
};

}  // namespace buddy_recorder

#endif  // BUDDY_RECORDER_THREAD_SAFE_EVENT_H