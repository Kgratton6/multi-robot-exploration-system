import { Module, Logger } from '@nestjs/common';
import { MongooseModule } from '@nestjs/mongoose';
import { ScheduleModule } from '@nestjs/schedule';
import { RobotModule } from './robot/robot.module';
import { LogsModule } from './logs/logs.module';
import { MissionModule } from './mission/mission.module';
import { config } from 'dotenv';
import { RobotController } from './controllers/robot.controller';
import { GatewayModule } from './gateways/gateway.module';

config(); // Load environment variables

@Module({
  imports: [
    MongooseModule.forRoot(process.env.MONGODB_URI),
    ScheduleModule.forRoot(),
    RobotModule,
    LogsModule,
    MissionModule,
    GatewayModule,
  ],
  controllers: [RobotController],
  providers: [Logger],
})
export class AppModule {}
