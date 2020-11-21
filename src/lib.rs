/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod srv;
mod steering;


////////////////////////////////////////////////////////////////////////////////


use rosrust::Time;
use rosrust_msg::std_msgs::Float64;
use rosrust_msg::geometry_msgs::PoseWithCovarianceStamped;
use rustros_tf::TfListener;
use rustros_tf::msg::std_msgs::Header;
use rustros_tf::msg::geometry_msgs::TransformStamped;

use errors::*;
use steering::{SteeringPidController, PidConstants};


pub fn run() -> Result<()> {

    let path = srv::get_static_path()?;

    // Get parameters
    let pid_constants = PidConstants {
        kp: 10.0,
        ki: 0.0,
        kd: 40.0,
        p_limit: 5.0,
        i_limit: 0.0,
        d_limit: 5.0,
    };

    // Setup controller
    let mut controller = SteeringPidController::new(path, pid_constants);

    // Setup publisher
    let steering_pub = rosrust::publish("/tiger_car/steer", 100)?;

    // Listen for transforms
    let listener = TfListener::new();
    let rate = rosrust::rate(100.0);
    let mut last_tf = TransformStamped::default();

    std::thread::sleep(std::time::Duration::new(1, 0));

    while rosrust::is_ok() {

        // Get updated odom transform
        let tf = listener.lookup_transform("map", "base_front", Time::new()).unwrap(); // Probably shouldn't unwrap this

        if tf != last_tf {

            // Save old last tf
            last_tf = tf.clone();

            // Determine new output from controller
            let output = controller.update_tf(tf);

            // Publish new control output
            steering_pub.send(output)?;
        }

        // Sleep to maintain rate
        rate.sleep();
    }

    // Breaks when a shutdown signal is sent
    rosrust::spin();

    Ok(())
}

