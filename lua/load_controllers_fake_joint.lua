
function load_controllers_fake_joint(depl, scheme, prefix)

  --[[ default arguments --]]
  local prefix = prefix or ""

  --[[ convenience vars --]]
  cp = rtt.Variable("ConnPolicy")
  depl:import("rtt_ros")
  gs:provides("ros"):import("lcsr_controllers")

  --[[ get required components --]]
  -- barrett_manager = depl:getPeer(prefix.."barrett_manager")
  -- effort_sum = depl:getPeer(prefix.."effort_sum")
  -- tf = depl:getPeer("tf")

  -- local wam = barrett_manager:getName()..".wam"

  --[[ get a port name from a task/port --]]
  local function port_name(task, port_name)
    if type(task) == "string" then
      task_name = task
    else
      task_name = task:getName()
    end

    return task_name.."."..port_name
  end

  --[[ simpler connect function --]]
  local function connect(from_task, from_port,to_task, to_port, policy)
    policy = policy or cp
    return depl:connect(
      port_name(from_task, from_port),
      port_name(to_task, to_port),
      policy);
  end

  --[[ load and get a component --]]
  local function loadComponent(name, component_type)
    depl:loadComponent(name, component_type);
    return depl:getPeer(name)
  end


  --[[ Create joint-space RML trajectory generator --]]
  traj_rml_name = prefix.."traj_rml"
  depl:loadComponent(traj_rml_name,"lcsr_controllers::JointTrajGeneratorRML");
  traj_rml = depl:getPeer(traj_rml_name)
  connect(traj_rml, "joint_position_out", traj_rml, "joint_position_in");
  connect(traj_rml, "joint_velocity_out", traj_rml, "joint_velocity_in");

  -- connect(wam, "position_out", traj_rml, "joint_position_in");
  -- connect(wam, "velocity_out", traj_rml, "joint_velocity_in");
  -- connect(                     traj_rml, "joint_position_out", pid, "joint_position_cmd_in");
  -- connect(                     traj_rml, "joint_velocity_out", pid, "joint_velocity_cmd_in");


  --[[ Configure all components --]]

  traj_rml:configure();

  --[[ Add to the conman scheme --]]
  scheme:addPeer(traj_rml);

  --[[ Add blocks to the scheme --]]
  scheme:addBlock(traj_rml_name);

  --[[ Create joint control group --]]
  joint_control = prefix.."joint_control"
  scheme:addGroup(joint_control);
  scheme:addToGroup(traj_rml_name, joint_control);

end

