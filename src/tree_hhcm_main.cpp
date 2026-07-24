#include <behaviortree_cpp/bt_factory.h>

#include <tree_hhcm/common/common.h>
#include <tree_hhcm/common/config_value.h>
#include <tree_hhcm/ros/pause_server.h>
#include <tree_hhcm/common/mission_group.h>

#include <iostream>
#include <string>
#include <getopt.h>
#include <vector>
#include <csignal>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
volatile bool g_running = true;

void sigint_handler(int signum)
{
    std::cout << "\nSIGINT received, shutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    // remove all arguments after '--ros-args'
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--ros-args")
        {
            argc = i;
            break;
        }
    }

    // Register SIGINT handler
    std::signal(SIGINT, sigint_handler);
    // argument parser
    std::string tree_path;
    double rate = 100.0;
    std::string name = "tree_main";
    std::vector<std::string> plugins;
    std::string xacro_args;
    std::vector<std::string> params;

    bool dont_sync = false;
    bool use_xacro = false;

    const char *const short_opts = "hr:n:p:da:m:";
    const option long_opts[] = {
        {"help", no_argument, nullptr, 'h'},
        {"rate", required_argument, nullptr, 'r'},
        {"name", required_argument, nullptr, 'n'},
        {"plugins", required_argument, nullptr, 'p'},
        {"dont-sync", no_argument, nullptr, 'd'},
        {"xacro-args", required_argument, nullptr, 'a'},
        {"param", required_argument, nullptr, 'm'},
        {nullptr, 0, nullptr, 0}};

    int opt;
    while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1)
    {
        switch (opt)
        {
        case 'h':
            std::cout << "Usage: " << argv[0] << " [--rate <rate>] [--name <name>] [--plugins <plugin>] [--param <param>] [--dont-sync] [--xacro-args <arg>] <tree_path>\n";
            std::cout << "       --plugins/-p can be specified multiple times.\n";
            std::cout << "       --param/-m can be specified multiple times.\n";
            std::cout << "       --xacro-args/-a can be specified multiple times.\n";
            std::cout << "       --dont-sync/-d disables time synchronization.\n";
            return 0;
        case 'r':
            rate = std::stod(optarg);
            break;
        case 'n':
            name = std::string(optarg);
            break;
        case 'p':
            plugins.push_back(std::string(optarg));
            break;
        case 'd':
            dont_sync = true;
            break;
        case 'a':
            xacro_args += " \"" + std::string(optarg) + "\"";
            break;
        case 'm':
            params.push_back(std::string(optarg));
            break;
        default:
            std::cerr << "Unknown option. Use --help for usage.\n";
            return 1;
        }
    }

    if (optind < argc)
    {
        tree_path = std::string(argv[optind]);
    }
    else
    {
        std::cerr << "Error: tree_path argument is required.\n";
        std::cout << "Usage: " << argv[0] << " [--rate <rate>] [--name <name>] [--plugins <plugin>] [--param <param>] [--dont-sync] [--xacro-args <arg>] <tree_path>\n";
        std::cout << "       --plugins/-p can be specified multiple times.\n";
        std::cout << "       --param/-m can be specified multiple times.\n";
        std::cout << "       --xacro-args/-a can be specified multiple times.\n";
        std::cout << "       --dont-sync/-d disables time synchronization.\n";
        return 1;
    }

    // fill globals
    // Get directory name from tree_path
    std::string tree_dirname = tree_path;
    auto pos = tree_dirname.find_last_of("/\\");
    if (pos != std::string::npos)
    {
        tree_dirname = tree_dirname.substr(0, pos);
    }
    else
    {
        tree_dirname = ".";
    }

    tree::Globals::instance().tree_dirname = tree_dirname;
    tree::Globals::instance().tree_dt = 1. / rate;

    tree::Printer p(name);
    p.cout() << "name: " << name << "\n";
    p.cout() << "tree_path: " << tree_path << "\n";
    if (!plugins.empty())
    {
        p.cout() << "plugins: ";
        for (const auto &plugin : plugins)
        {
            std::cout << plugin << ", ";
        }
        std::cout << "\n";
    }
    if (!xacro_args.empty())
    {
        p.cout() << "xacro_args: " << xacro_args << "\n";
    }
    p.cout() << "rate: " << rate << " Hz\n";

    // preprocess xacro if required
    xacro_args += " tree_dirname:=" + tree_dirname;
    std::string tree_string = tree::Globals::instance().check_output("xacro " + tree_path + xacro_args);

    // main logic
    BT::BehaviorTreeFactory factory;
    for (auto plugin : plugins)
    {
        p.cout() << "Loading plugin: " << plugin << "\n";
        factory.registerFromPlugin(plugin);
    }

    // build tree
    auto blackboard = BT::Blackboard::create();
    tree::ConfigValueBase::root_tree_blackboard = blackboard;
    auto tree = factory.createTreeFromText(tree_string, blackboard);

    // override cli params to blackboard
    for (const auto &param : params)
    {
        // a param is in the form key:=value
        auto delim_pos = param.find(":=");

        if (delim_pos == std::string::npos)
        {
            throw std::runtime_error("Invalid param format: " + param + ". Expected key:=value");
        }

        std::string key = param.substr(0, delim_pos);
        std::string value_str = param.substr(delim_pos + 2);

        // type is autodetected

        // bool?
        if (value_str == "true" || value_str == "false")
        {
            bool value = (value_str == "true");
            blackboard->set<bool>(key, value);
            p.cout() << "Set param (bool) " << key << " := " << (value ? "true" : "false") << "\n";
            continue;
        }

        // int?
        try
        {
            size_t idx = 0;
            int value = std::stoi(value_str, &idx);
            // ensure whole string was converted
            if (idx < value_str.size())
            {
                throw "";
            }
            blackboard->set<int>(key, value);
            p.cout() << "Set param (int) " << key << " := " << value << "\n";
            continue;
        }
        catch (...)
        {
        }

        // double?
        try
        {
            size_t idx = 0;
            int value = std::stod(value_str, &idx);
            // ensure whole string was converted
            if (idx < value_str.size())
            {
                throw "";
            }
            blackboard->set<double>(key, value);
            p.cout() << "Set param (double) " << key << " := " << value << "\n";
            continue;
        }
        catch (...)
        {
        }

        // string
        blackboard->set<std::string>(key, value_str);
        p.cout() << "Set param (string) " << key << " := " << value_str << "\n";
    }

    // pause/resume service
    tree::PauseServer pause_server(name);

    // warn about ungrouped nodes
    for (const auto &path : tree::find_ungrouped_nodes(tree))
    {
        p.cerr() << "warning: [" << path << "] is not inside any MissionGroup\n";
    }

    // run tree until ctrl+c
    std::chrono::duration<double> dt(1.0 / rate);
    std::chrono::duration<double> stats_dt(2.0);
    auto t_report_next = std::chrono::steady_clock::now() + stats_dt;
    int niters = 0;
    bool success = false;

    while (g_running)
    {
        auto t_next = std::chrono::steady_clock::now() + dt;

        // while paused, skip ticking: node-internal time advances by
        // tree_dt per tick, so the whole mission freezes in place
        if (pause_server.paused())
        {
            niters = 0;
            t_report_next = std::chrono::steady_clock::now() + stats_dt;
            std::this_thread::sleep_until(t_next);
            continue;
        }

        // tick the tree
        auto status = tree.tickOnce();
        niters++;

        // report
        if (std::chrono::steady_clock::now() >= t_report_next)
        {
            p.cout() << "Running at " << niters / stats_dt.count() << " Hz\n";
            niters = 0;
            t_report_next += stats_dt;
        }

        // handle tree result
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            p.cout() << "Tree finished with status " << toStr(status) << "\n";
            success = status == BT::NodeStatus::SUCCESS;
            break;
        }

        if (!dont_sync)
        {
            std::this_thread::sleep_until(t_next);
        }
    }

    tree::ConfigValueBase::root_tree_blackboard.reset();
    tree.~Tree();
    blackboard.reset();

    _Exit(success ? 0 : 1);
}