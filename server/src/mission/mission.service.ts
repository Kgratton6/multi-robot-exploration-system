import { Injectable } from '@nestjs/common';
import { Mission } from '../interfaces/mission.interface';
import { v4 as uuidv4 } from 'uuid';

@Injectable()
export class MissionService {
    private missions: Map<string, Mission> = new Map();
    private currentMission: Mission | null = null;

    async startMission(robotIds: string[]): Promise<Mission> {
        if (this.currentMission) {
            throw new Error('A mission is already in progress');
        }

        const mission: Mission = {
            id: uuidv4(),
            startTime: new Date().toISOString(),
            status: 'ongoing',
            robots: robotIds,
            logs: []
        };

        this.currentMission = mission;
        this.missions.set(mission.id, mission);

        return mission;
    }

    async stopMission(robotIds: string[]): Promise<void> {
        if (!this.currentMission) {
            throw new Error('No mission in progress');
        }

        // Terminer la mission si tous les robots sont arrêtés
        const remainingActiveRobots = this.currentMission.robots.filter(
            robotId => !robotIds.includes(robotId)
        );

        if (remainingActiveRobots.length === 0) {
            this.currentMission.status = 'completed';
            this.currentMission.endTime = new Date().toISOString();
            this.currentMission = null;
        }
    }

    async getMissions(): Promise<Mission[]> {
        return Array.from(this.missions.values());
    }
}