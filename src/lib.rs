/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod srv;
mod steering;


////////////////////////////////////////////////////////////////////////////////


use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::PoseWithCovarianceStamped;

use errors::*;


pub fn run() -> Result<()> {

    let path = srv::get_static_path()?;

    // Get parameters
    let pid_constants = steering::PidConstants {
        kp: rosrust::param("~kp").unwrap().get()?,
        ki: rosrust::param("~ki").unwrap().get()?,
        kd: rosrust::param("~kd").unwrap().get()?,
        p_limit: rosrust::param("~p_limit").unwrap().get()?,
        i_limit: rosrust::param("~i_limit").unwrap().get()?,
        d_limit: rosrust::param("~d_limit").unwrap().get()?,
    };

    let speed: f64 = rosrust::param("~speed").unwrap().get()?;
    let speed = Float64 { data: speed };

    // Setup controller
    let controller = steering::SteeringPidController::new(path, pid_constants);

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/control/drivetrain", 100)?;
    let steering_pub = rosrust::publish("/tiger_car/control/steering", 100)?;

    // Register Subscriber
    rosrust::subscribe(
        "amcl_pose",
        100,
        move |pose: PoseWithCovarianceStamped| {
            
            rosrust::ros_info!("Pose Received: {:?}", pose);
            
            let steering = controller.update(pose);

            drivetrain_pub.send(speed.clone()).unwrap();
            steering_pub.send(steering).unwrap();
        }
    )?;

    // Breaks when a shutdown signal is sent
    rosrust::spin();

    Ok(())
}

