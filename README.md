# Veesion

## Veesion Project Docker Setup

Veesion Docker container is designed to provide a ready-to-use development environment for the Veesion project, based on ROS 2 Humble. It includes all necessary tools and dependencies such as OpenVINS for visual-inertial navigation and imu_tools for IMU data processing. The container sets up a ROS 2 workspace (veesion_ws), installs required packages and builds OpenVINS using colcon. With the included Dockerfile and setup script, users can quickly clone the repository, build the workspace, and start developing in an isolated, reproducible environment tailored for ROS 2 applications. During the setup, it sources install/setup.bash for OpenVINS so you do not need to write it every time you open a terminal.

This guide will walk you through the process of setting up the development environment for the Veesion project using Docker and Visual Studio Code (VSCode). Follow the steps carefully to clone the repository, open it in VSCode, and build the container environment.

## Prerequisites

Before starting, ensure you have the following installed:

1. **Docker** – This project requires Docker to build and run the development container.
- To install docker and set the correct user rights please use the following commands:
  ```bash
  sudo apt install docker.io git python3-pip
  pip3 install vcstool
  echo export PATH=$HOME/.local/bin:$PATH >> ~/.bashrc
  source ~/.bashrc
  sudo groupadd docker
  sudo usermod -aG docker $USER
  newgrp docker
  ```
- Now you can check if the installation was successful by running the following command:
  ```bash
  docker run hello-world
  ```
  
2. **Visual Studio Code (VSCode)** – A lightweight code editor that supports Docker integration.
   
3. **Remote Development Extension for VSCode** – This VSCode extension allows you to work with Docker containers directly in VSCode.
- Install the extension: Within VS Code search in Extensions (CTRL+SHIFT+X) for the “Remote Development” Extension and install it.

## Step-by-Step Setup Guide

### 1. Open VSCode and Navigate to Your Home Directory

- Launch Visual Studio Code.
- Open the terminal in VSCode and change the directory to your home directory by running the following command:
  ```bash
  cd ~

### 2. Clone the Repository
- In the terminal, clone the repository containing the project into your home directory by running:
  ```bash
  git clone https://github.com/BirkanG/veesion.git
- This will create a folder named veesion in your home directory, containing the project files.

### 3. Open the veesion Folder in VSCode
- Once the cloning is complete, open the veesion directory using UI or with the following commands:
  ```bash
  cd veesion
  code .
  ```
### 4. Reopen in Container (VSCode Remote Development)
- If you have the Remote Development extension installed in VSCode, you should see a pop-up in the bottom right corner of VSCode asking if you want to reopen the project in a container. It will look something like: "Reopen in Container"

  ![Screenshot from 2024-12-02 00-57-46](https://github.com/user-attachments/assets/e1034c14-a1e2-4d54-a939-4cdecd374b9e)

- Click on this pop-up, and VSCode will automatically build the Docker container based on the Dockerfile in the repository.
- If it does not show up, use Ctrl+Shift+P and write Dev Containers: Reopen in Container.
  
### 5. Start Developing
- The Docker container will now be built, which may take a few minutes. Once the container is built and running, VSCode will automatically reopen the project inside the container. You are now set up to work in the isolated environment provided by Docker.
- Once the container is up and running, you can start working on the project just like you would in a normal VSCode environment.

### 6. Test Container
- Install the ROS bag for OpenVINS in our drive and play it to test if everything works correctly.
  ```bash
  ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
- Go inside the openvins_test dir that you downloaded and play the bag:
    ```bash
    ros2 bag play v1_ros2.db3
