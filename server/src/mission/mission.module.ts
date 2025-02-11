import { Module } from '@nestjs/common';
import { MissionController } from './mission.controller';
import { MissionService } from './mission.service';
import { RobotGateway } from '../gateways/robot.gateway';

@Module({
    imports: [],
    controllers: [MissionController],
    providers: [MissionService, RobotGateway],
    exports: [MissionService] // Exporté pour être utilisé par RobotSimulator
})
export class MissionModule {}