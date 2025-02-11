import { Controller, Post, Logger } from '@nestjs/common';
import { MissionService } from 'src/mission/mission.service';

@Controller('robots')
export class RobotController {
  private readonly logger = new Logger(RobotController.name);

  constructor(private readonly missionService: MissionService) {}

  @Post('mission/start')
  async startMission(): Promise<{ message: string }> {
    this.logger.log('Received request to start mission');
    return await this.missionService.startMission();
  }

  @Post('mission/stop')
  async stopMission(): Promise<{ message: string }> {
    this.logger.log('Received request to stop mission');
    return await this.missionService.stopMission();
  }

  @Post('identify')
  async identify(): Promise<{ message: string }> {
    this.logger.log('Received request to identify robot');
    return await this.missionService.identify();
  }
}