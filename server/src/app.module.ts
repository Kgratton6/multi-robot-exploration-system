import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { MissionModule } from './mission.module'; //Makes  MissionModule available to the whole application. The API should be available at: GET http://localhost:3000/missions/status POST http://localhost:3000/missions/send
@Module({
  imports: [MissionModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
