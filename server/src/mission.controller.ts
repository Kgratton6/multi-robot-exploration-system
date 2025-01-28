//Controller that exposes a REST API for interacting with missions
//POST /missions/send => to send missions   GET /missions/status => To check if the service is running
import { Controller, Get, Post, Body } from '@nestjs/common';
import { MissionService } from './mission.service';

@Controller('missions')
export class MissionController {
  constructor(private readonly missionService: MissionService) {}

  @Post('send')
  sendMission(@Body() missionData: any) {
    console.log('Received mission:', missionData);
    return { message: 'Mission received!', data: missionData };
  }

  @Get('status')
  getMissionStatus() {
    return { status: 'Service is running' };
  }
}
