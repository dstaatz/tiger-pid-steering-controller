/* Copyright (C) 2020 Dylan Staatz - All Rights Reserved. */


use std::time::Duration;

use rosrust;
use rosrust_msg::geometry_msgs::PoseStamped;
use rosrust_msg::nav_msgs::{Path, GetPlan, GetPlanReq};

use crate::errors::*;


pub fn get_static_path() -> Result<Path> {
    get_path("/static_path").chain_err(|| "Failed to get static path")
}

fn get_path(service_name: &str) -> Result<Path> {

    // Wait for service
    rosrust::ros_info!("Waiting for {} service", service_name);
    rosrust::wait_for_service(service_name, Some(Duration::from_secs(10)))?;

    // Create client
    let client = rosrust::client::<GetPlan>(service_name)?;

    let request = GetPlanReq {
        start: PoseStamped::default(),
        goal: PoseStamped::default(),
        tolerance: 0.5,
    };

    Ok(client.req(&request)??.plan)
}

