/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use std::time::Duration;

use rosrust;
use rosrust_msg::*;

use crate::errors::*;


pub fn get_static_map() -> Result<nav_msgs::OccupancyGrid> {
    get_map("static_map").chain_err(|| "Failed to get static map")
}

fn get_map(service_name: &str) -> Result<nav_msgs::OccupancyGrid> {

    // Wait for service
    rosrust::ros_info!("Waiting for {} service", service_name);
    rosrust::wait_for_service(service_name, Some(Duration::from_secs(10)))?;

    // Create client
    let client = rosrust::client::<nav_msgs::GetMap>(service_name)?;

    let req = nav_msgs::GetMapReq {};

    Ok(client.req(&req)??.map)
}


pub fn get_static_path() -> Result<nav_msgs::Path> {
    get_path("static_path").chain_err(|| "Failed to get static path")
}

fn get_path(service_name: &str) -> Result<nav_msgs::Path> {

    // Wait for service
    rosrust::ros_info!("Waiting for {} service", service_name);
    rosrust::wait_for_service(service_name, Some(Duration::from_secs(10)))?;

    // Create client
    let client = rosrust::client::<nav_msgs::GetPlan>(service_name)?;

    let request = nav_msgs::GetPlanReq {
        start: geometry_msgs::PoseStamped::default(),
        goal: geometry_msgs::PoseStamped::default(),
        tolerance: 0.5,
    };

    Ok(client.req(&request)??.plan)
}

