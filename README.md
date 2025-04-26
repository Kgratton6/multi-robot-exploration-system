# 🤖 Multi‑Robot Exploration System

## Project Overview

This project, carried out for the INF3995 course at Polytechnique Montréal, aims to design a complete autonomous multi‑robot exploration system. Inspired by a space‑mission scenario, it simulates a terrain‑exploration mission with two AgileX Limo robots. The system is divided into three main parts:

- **Physical robots**: ROS 2 Humble software
- **Ground station**: NestJS server, MongoDB database, Angular interface
- **Simulation**: Gazebo Fortress to test without the physical robots

The operator interacts through a unified web interface to start, supervise, and stop missions. All components, except the embedded code, are containerised with Docker.

## 🌟 Robot Features

- Start/stop missions from the web interface
- Autonomous navigation with obstacle avoidance
- Real‑time 2D mapping visible in the interface
- Mission data saved in a MongoDB database
- Direct inter‑robot communication (P2P)

## 🗂️ Project Structure

```
.
├── client/                     # Angular user interface
├── server/                     # NestJS server
├── robot/
│   ├── common/                 # Files shared between robots (sound, images, ...)
│   ├── gazebo/                 # Gazebo models and world
│   ├── gazebo_launch_scripts/  # Scripts to launch the simulation separately
│   ├── limo/                   # Limo‑specific robot code
│   ├── limo_launch_scripts/    # Scripts to launch the limos separately
│   ├── robot/                  # On‑board robot behaviour (navigation, communication, identification, ...)
│   ├── robot_launch_scripts/   # Scripts to launch on‑board behaviour separately
│   └── utilities/              # Installation and debugging scripts
├── docker-compose.yml          # Ground station launch with Docker
├── Dockerfile                  # Container build
├── start_base.sh               # Ground‑station start script (Frontend, Backend)
├── start_docker.sh             # Launches the complete environment with Docker
├── start_gazebo.sh             # Launches Gazebo simulation (Frontend, Backend, Gazebo)
├── start_install.sh            # Installs required dependencies
├── START.md                    # Usage instructions (see below)
```

## 📸 Project Images

### Web Interface
![frontend](robot/common/readme_img_front.png)

### Map Generation
![map](robot/common/readme_img_map.png)

### Database
![database](robot/common/readme_img_database.png)

### Gazebo Simulation
![Gazebo](robot/common/readme_img_gazebo.png)

## ⚙️ Installation and Execution

Please refer to the `START.md` file at the root of the project. It contains:

- Prerequisites for Ubuntu 22.04 (or WSL2)
- Commands to launch the project with `docker-compose`
- Instructions for a local launch (outside Docker)
- Details to run the Gazebo simulation with or without robots

Quick link: [START.md](./START.md)

## 👥 Team 102
Zerouali, Amine  
Gratton Fournier, Kevin Santiago  
Haddadi, Issam  
Hachemi Boumila, Rafik  
Abassi, Yassine Mohamed Taha  
Milord, Mario Junior

## 🛠️ Technologies Used


<img  align="left" width="50" src="https://raw.githubusercontent.com/marwin1991/profile-technology-icons/refs/heads/main/icons/gitlab.png" alt="Ros 2" title="Ros 2"/>
<img  align="left" width="50" src="https://user-images.githubusercontent.com/25181517/183890595-779a7e64-3f43-4634-bad2-eceef4e80268.png" alt="Angular" title="Angular"/>
<img  align="left" width="50" src="https://user-images.githubusercontent.com/25181517/192158954-f88b5814-d510-4564-b285-dff7d6400dad.png" alt="HTML" title="HTML"/>
<img align="left"  width="50" src="https://user-images.githubusercontent.com/25181517/183898674-75a4a1b1-f960-4ea9-abcb-637170a00a75.png" alt="CSS" title="CSS"/>
<img  align="left" width="50" src="https://user-images.githubusercontent.com/25181517/183890598-19a0ac2d-e88a-4005-a8df-1ee36782fde1.png" alt="TypeScript" title="TypeScript"/>
<img align="left"  width="50" src="https://user-images.githubusercontent.com/25181517/183423507-c056a6f9-1ba8-4312-a350-19bcbc5a8697.png" alt="Python" title="Python"/>
<img  align="left" width="50" src="https://raw.githubusercontent.com/marwin1991/profile-technology-icons/refs/heads/main/icons/mongodb.png" alt="Ros 2" title="Ros 2"/>
<img  align="left" width="50" src="https://raw.githubusercontent.com/marwin1991/profile-technology-icons/refs/heads/main/icons/linux.png" alt="Ros 2" title="Ros 2"/>
