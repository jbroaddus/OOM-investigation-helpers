#include <thread_safe_event.h>

namespace buddy_recorder
{
ThreadSafeEvent& ThreadSafeEvent::operator=(std::string event)
{
  std::scoped_lock lock{ mutex_ };
  event_str_ = std::move(event);

  return *this;
}

std::string ThreadSafeEvent::getEvent()
{
  std::scoped_lock lock{ mutex_ };
  return event_str_;
}

}  // namespace buddy_recorder