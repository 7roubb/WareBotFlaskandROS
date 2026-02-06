# WareBot Project

This project integrates a Flask-based backend with ROS 2 for managing and simulating warehouse robots. It includes a web interface for monitoring and control, and a set of ROS 2 packages for robot simulation and task management.

## Project Structure

*   **backend/**: Flask application providing API, WebSocket, and database interactions.
*   **warebot_map_merger**, **warebot_robot_bridge**, **warebot_task_runner**: ROS 2 packages for robot operations.
*   **docker-compose.yml**: Orchestrates the backend services (API, MongoDB, HiveMQ, MinIO, InfluxDB, Grafana).

## Setup Instructions

### 1. Backend Setup

The backend and its dependencies are containerized using Docker.

1.  **Prerequisites**: Ensure [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) are installed.
2.  **Start Services**:
    Run the following command in the root directory of this repository:
    ```bash
    docker-compose up --build -d
    ```
    This will start:
    *   **API**: `http://localhost:5000`
    *   **MongoDB**: `localhost:27017`
    *   **HiveMQ** (MQTT): `localhost:1884` (Websocket), `8080` (Dashboard)
    *   **MinIO** (S3): `http://localhost:9000` (Console: `9001`)
    *   **InfluxDB**: `http://localhost:8086`
    *   **Grafana**: `http://localhost:3000`

### 2. Frontend Setup

The frontend is a separate repository that needs to be cloned and run.

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/7roubb/WareBotFrontend.git
    cd WareBotFrontend
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```

3.  **Run the Application**:
    ```bash
    npm run dev
    ```
    The frontend should now be accessible in your browser (usually at `http://localhost:5173`).

### 3. ROS 2 Setup

The ROS 2 packages need to be moved to your ROS 2 workspace (e.g., Jazzy).

1.  **Copy Packages**:
    Copy the following folders from this repository to your ROS 2 workspace `src` directory (e.g., `~/ros2_ws/src/`):
    *   `warebot_map_merger`
    *   `warebot_robot_bridge`
    *   `warebot_task_runner`

2.  **Build Workspace**:
    Navigate to your workspace root and build the packages:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3.  **Source Workspace**:
    Source the setup file to use the packages:
    ```bash
    source install/setup.bash
    ```
