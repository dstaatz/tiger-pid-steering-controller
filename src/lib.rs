/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;

mod errors;
mod srv;


////////////////////////////////////////////////////////////////////////////////


use rosrust_msg::std_msgs;

use errors::*;


pub fn run() -> Result<()> {

    let _path1 = srv::get_static_map()?;
    let _path2 = srv::get_static_path()?;

    // Create publishers
    let drivetrain_pub = rosrust::publish("/tiger_car/control/drivetrain", 100)?;
    let steering_pub = rosrust::publish("/tiger_car/control/steering", 100)?;

    // Create object that maintains 10Hz between sleep requests
    let rate = rosrust::rate(10.0);

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {

        let zero = std_msgs::Float64 { data: 0.0 };

        drivetrain_pub.send(zero.clone())?;
        steering_pub.send(zero)?;

        // Sleep to maintain rate
        rate.sleep();
    }

    Ok(())
}

