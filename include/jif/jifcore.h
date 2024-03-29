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
            if (topic.empty())
            {
                std::cerr << "[JIFCore] Warning: no publisher has been created due to empty topic string" << std::endl;
                return;
            }
            m_Publishers[topic] = create_publisher<T>(topic, 10);
        }

        template <typename T>
        void RegisterSubscription(const std::string &topic, const std::function<void(const T &msg)> &callback)
        {
            if (topic.empty())
            {
                std::cerr << "[JIFCore] Warning: no subscription has been created due to empty topic string" << std::endl;
                return;
            }
            m_Subsciptions[topic] = create_subscription<T>(topic, 10, callback);
        }

        template <typename T>
        void Publish(const std::string &topic, const T &message)
        {
            if (topic.empty())
            {
                std::cerr << "[JIFCore] Warning: message has not been sent due to empty topic string" << std::endl;
                return;
            }
            auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(m_Publishers[topic]);
            publisher->publish(message);
        }

        void UnregisterPublisher(const std::string &topic)
        {
            if (topic.empty())
            {
                std::cerr << "[JIFCore] Warning: no publisher has been removed due to empty topic string" << std::endl;
                return;
            }
            m_Publishers[topic].reset();
        }
        
        void UnregisterSubscription(const std::string &topic)
        {
            if (topic.empty())
            {
                std::cerr << "[JIFCore] Warning: no subscription has been removed due to empty topic string" << std::endl;
                return;
            }
            m_Subsciptions[topic].reset();
        }

    private:
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> m_Publishers;
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> m_Subsciptions;
    };
    typedef std::shared_ptr<JIFCore> JIFCorePtr;
}
