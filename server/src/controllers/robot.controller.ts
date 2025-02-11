import { Controller, Post, Logger } from '@nestjs/common';

@Controller('robots')
export class RobotController {
    private readonly logger = new Logger(RobotController.name);

    @Post('mission/start')
    async startMission(): Promise<void> {
        // Simulation de la commande ROS2
        this.logger.log('Would execute: ros2 topic pub /messages std_msgs/msg/String \'{data: "{\\"action\\": \\"start_mission\\"}"}\' -1');
    }

    @Post('mission/stop')
    async stopMission(): Promise<void> {
        // Simulation de la commande ROS2
        this.logger.log('Would execute: ros2 topic pub /messages std_msgs/msg/String \'{data: "{\\"action\\": \\"end_mission\\"}"}\' -1');
    }
}