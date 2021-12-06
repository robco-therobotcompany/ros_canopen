#include <canopen_motor_node/motor_chain.h>
#include <canopen_motor_node/handle_layer.h>
#include <socketcan_interface/xmlrpc_settings.h>
using namespace canopen;

MotorChain::MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv) :
        RosChain(nh, nh_priv), motor_allocator_("canopen_402", "canopen::MotorBase::Allocator") {}

bool MotorChain::nodeAdded(XmlRpc::XmlRpcValue &params, const canopen::NodeSharedPtr &node, const LoggerSharedPtr &logger)
{
    // Get motor list for node
    XmlRpc::XmlRpcValue motors;
    if(!params.hasMember("motors")){
        ROS_ERROR("motor list not found!");
        return false;
    }

    motors = params["motors"];
    
    // Convert from XMLRPC array to struct if necessary
    if(motors.getType() ==  XmlRpc::XmlRpcValue::TypeArray){
        XmlRpc::XmlRpcValue new_struct;
        for(size_t i = 0; i < motors.size(); ++i){
            if(motors[i].hasMember("name")){
                std::string &name = motors[i]["name"];
                new_struct[name] = motors[i];
                ROS_INFO_STREAM("Found motor '" << name << "'");
            }else{
                ROS_ERROR_STREAM("Motor at list index " << i << " has no name");
                return false;
            }
        }
        motors = new_struct;
    }

    // Add all motors
    for(XmlRpc::XmlRpcValue::iterator it = motors.begin(); it != motors.end(); ++it){
        std::string name = it->first;
        XmlRpc::XmlRpcValue currentMotor = it->second;
        std::string &joint = name;
        if(currentMotor.hasMember("joint")) joint.assign(currentMotor["joint"]);

        ROS_INFO_STREAM("Adding motor '" << name << "' for joint '" << joint << "'");

        if(!robot_layer_->getJoint(joint)){
            ROS_ERROR_STREAM("joint " + joint + " was not found in URDF");
            return false;
        }

        std::string alloc_name = "canopen::Motor402::Allocator";
        if(currentMotor.hasMember("motor_allocator")) alloc_name.assign(currentMotor["motor_allocator"]);

        XmlRpcSettings settings;
        int cmd_offset = 0;
        if(currentMotor.hasMember("motor_layer")) {
            settings = currentMotor["motor_layer"];
            cmd_offset = settings.get_optional("cmd_offset", 0);
            ROS_INFO("Found cmd_offset %i for motor %s", cmd_offset, name.c_str());
        }

        MotorBaseSharedPtr motor;

        try{
            motor = motor_allocator_.allocateInstance(alloc_name, name + "_motor", node->getStorage(), settings);
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            return false;
        }

        if(!motor){
            ROS_ERROR_STREAM("Could not allocate motor '" << name << "'.");
            return false;
        }

        MergedXmlRpcStruct merged;
        merged = MergedXmlRpcStruct(params, merged);
        if (currentMotor.hasMember("defaults")) {
          merged = MergedXmlRpcStruct(currentMotor["defaults"], merged);
        }

        motor->registerDefaultModes(node->getStorage(), cmd_offset);
        motors_->add(motor);
        logger->add(motor);

        HandleLayerSharedPtr handle = std::make_shared<HandleLayer>(joint, motor, node->getStorage(), merged);

        canopen::LayerStatus s;
        if(!handle->prepareFilters(s)){
            ROS_ERROR_STREAM(s.reason());
            return false;
        }

        robot_layer_->add(joint, handle);
        logger->add(handle);

        ROS_INFO_STREAM("Added motor '" << name << "' for joint '" << joint << "'");
    }

    return true;
}

bool MotorChain::setup_chain() {
    motors_.reset(new LayerGroupNoDiag<MotorBase>("402 Layer"));
    robot_layer_.reset(new RobotLayer(nh_));

    ros::Duration dur(0.0) ;

    if(RosChain::setup_chain()){
        add(motors_);
        add(robot_layer_);

        if(!nh_.param("use_realtime_period", false)){
            dur.fromSec(boost::chrono::duration<double>(update_duration_).count());
            ROS_INFO_STREAM("Using fixed control period: " << dur);
        }else{
            ROS_INFO("Using real-time control period");
        }
        cm_.reset(new ControllerManagerLayer(robot_layer_, nh_, dur));
        add(cm_);

        return true;
    }

    return false;
}
