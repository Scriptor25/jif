/*
 * ------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Felix Schreiber wrote this code. As long as you retain this
 * notice, you can do whatever you want with this stuff. If we
 * meet someday, and you think this stuff is worth it, you can
 * buy me a beer in return.
 * ------------------------------------------------------------
 */

#pragma once

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace jif
{
    class JIFCore : public rclcpp::Node
    {
    public:
        JIFCore()
            : Node("jifcore")
        {
        }

        template <typename T>
        void RegisterPublisher(const std::string &topic)
        {
            m_Publishers[topic] = create_publisher<T>(topic, 10);
        }

        template <typename T>
        void RegisterSubscription(const std::string &topic, const std::function<void(const T &msg)> &callback)
        {
            m_Subsciptions[topic] = create_subscription<T>(topic, 10, callback);
        }

        template <typename T>
        void Publish(const std::string &topic, const T &message)
        {
            auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(m_Publishers[topic]);
            publisher->publish(message);
        }

    private:
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> m_Publishers;
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> m_Subsciptions;
    };
    typedef std::shared_ptr<JIFCore> JIFCorePtr;
}
