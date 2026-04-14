#pragma once

#include <chrono>
#include <string>
#include <expected>
#include <optional>

template <typename T>
concept ChronoDuration = requires {
  typename T::rep;
  typename T::period;
  requires std::is_same_v<T, std::chrono::duration<typename T::rep, typename T::period>>;
};

enum class TimedBoxErrors
{
  Expired,
  NotSet
};

/**
 * @brief A box that holds a item for a certain lifespan
 * @tparam ItemT The item type
 */
template <typename ItemT>
class TimedBox
{
public:

  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

  struct ItemContainer
  {
    ItemT item;
    TimePoint set_time;
  };

  /**
 * @brief Construct a new TimedBox with no lifespan
 */
  explicit TimedBox()
  {
  }

  /**
   * @brief Construct a new TimedBox
   * @param lifespan The lifespan for the subscriber before it returns an Expired error
   */
  template <ChronoDuration DurationT>
  explicit TimedBox(
    const DurationT& lifespan)
  {
    set_lifespan(lifespan);
  }

  template <ChronoDuration DurationT>
  void set_lifespan(const DurationT & lifespan)
  {
    lifespan_ = std::chrono::duration_cast<std::chrono::milliseconds>(lifespan);
  }

  void set(const ItemT& item) {
      auto incoming_item = ItemContainer{item, std::chrono::system_clock::now()};
      item_ = incoming_item;
  }

  TimedBox& operator=(const ItemT& item) {
    set(item);
    return *this;
  }

  /**
   * @brief Get the item as an Expected, or reason why it is not available
   * @return The item or an error enum
   */
  std::expected<ItemT, TimedBoxErrors> value() const
  {
    if (!item_) {
      return std::unexpected(TimedBoxErrors::NotSet);
    }
    if (is_expired()) {
      return std::unexpected(TimedBoxErrors::Expired);
    }
    return item_.value().item;
  }

  /**
   * @brief Get the item or a default value if it is not available
   * @param default_value The default value to return if the item is not available
   * @return The item or the default value
   */
  ItemT value_or(const ItemT& default_value) const
  {
    auto val = value();
    if (val) {
      return val.value();
    }
    return default_value;
  }

  /**
   * @brief Get the time the item was last set.
   *
   * @return std::optional<TimePoint> Last set time
   */
  [[nodiscard]] std::optional<TimePoint> get_last_set_time() const
  {
    return item_ ? std::optional<TimePoint>(item_.value().set_time) : std::nullopt;
  }

  /**
   * @brief Check if the item is expired
   * @return True if the item is expired, or has never been set
   */
  [[nodiscard]] bool is_expired() const
  {
    if (!item_) {
      // if the item has never been set, consider it expired
      return true;
    }

    if (!lifespan_.has_value()) {
      // if no lifespan is set, the item should not be considered expired
      return false;
    }

    return is_expired(item_.value());
  }

protected:

  /**
   * @brief Check if the item is expired
   * @param item_container The item container
   * @return True if the item is expired
   */
  [[nodiscard]] bool is_expired(const ItemContainer & item_container) const
  {
    const auto time_since_last_item = std::chrono::system_clock::now() - item_container.set_time;
    return time_since_last_item > lifespan_;
  }

  std::optional<ItemContainer> item_;
  std::optional<std::chrono::milliseconds> lifespan_;
};

inline std::string to_string(TimedBoxErrors e)
{
  switch (e) {
    case TimedBoxErrors::Expired: return "Item lifespan expired";
    case TimedBoxErrors::NotSet:  return "Item has not been set";
    default:         return "Unknown TimedBox error";
  }
}
