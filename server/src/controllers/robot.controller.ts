import { Controller, Post, Get, Body, Param } from '@nestjs/common';
import { RobotState } from '../interfaces/robot.interface';

@Controller('robots')
export class RobotController {
    @Get('states')
    async getRobotStates(): Promise<RobotState[]> {
        // Temporairement, retourne un tableau vide en attendant l'intégration avec le vrai robot
        return [];
    }

    @Post(':robotId/identify')
    async identifyRobot(@Param('robotId') robotId: string): Promise<void> {
        // Temporairement, ne fait rien en attendant l'intégration avec le vrai robot
        return;
    }

    @Post('mission/start')
    async startMission(@Body() data: { robotIds: string[] }): Promise<void> {
        // Temporairement, ne fait rien en attendant l'intégration avec le vrai robot
        return;
    }

    @Post('mission/stop')
    async stopMission(@Body() data: { robotIds: string[] }): Promise<void> {
        // Temporairement, ne fait rien en attendant l'intégration avec le vrai robot
        return;
    }

    @Post('mission/return')
    async returnToBase(@Body() data: { robotIds: string[] }): Promise<void> {
        // Temporairement, ne fait rien en attendant l'intégration avec le vrai robot
        return;
    }

    @Post(':robotId/wheel-mode')
    async setWheelMode(
        @Param('robotId') robotId: string,
        @Body() data: { mode: 'ackerman' | 'differential' }
    ): Promise<void> {
        // Temporairement, ne fait rien en attendant l'intégration avec le vrai robot
        return;
    }

    @Post('p2p/enable')
    async enableP2P(@Body() data: { robotIds: string[] }): Promise<void> {
        // Pour l'instant, cette fonctionnalité n'est pas implémentée
        return;
    }

    @Post('p2p/disable')
    async disableP2P(@Body() data: { robotIds: string[] }): Promise<void> {
        // Pour l'instant, cette fonctionnalité n'est pas implémentée
        return;
    }
}