import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { MissionModule } from './mission/mission.module';
// import { RobotSimulatorModule } from './robot-simulator/robot-simulator.module';
import { RobotModule } from './robot/robot.module';

@Module({
  imports: [
    MissionModule,
    // RobotSimulatorModule // Module de simulation désactivé
    RobotModule
  ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
