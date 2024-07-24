#!/bin/bash

# Source the setup script
# source /home/jon/.local/share/ov/pkg/isaac_sim-2023.1.1/setup_python_env.sh
source /home/jon/.local/share/ov/pkg/isaac_sim-2023.1.1/setup_conda_env.sh

# Check if a script name is provided
if [ -z "$1" ]; then
  echo "No script name provided. Usage: ./your_script.sh <script_name.py>"
  exit 1
fi

# Get the script name from the first argument
SCRIPT_NAME=$1

# Run the specified Python script
#/home/jon/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh $SCRIPT_NAME --nucleus-server localhost/NVIDIA/Assets/Isaac/2022.2.0
python $SCRIPT_NAME --nucleus-server localhost/NVIDIA/Assets/Isaac/2022.2.0
