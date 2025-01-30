import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { BehaviorSubject, Observable, map } from 'rxjs';
import { environment } from '../../environments/environment';
import { Robot, RobotState, WheelMode } from '../models/robot.model';
import { WebSocketService } from './websocket.service';

@Injectable({
    providedIn: 'root'
})
export class RobotService {
    private robots$ = new BehaviorSubject<Robot[]>([]);

    constructor(
        private http: HttpClient,
        private wsService: WebSocketService
    ) {
        // Subscribe to robot state updates from WebSocket
        this.wsService.getRobotStates().subscribe(states => {
            const updatedRobots = states.map(state => ({
                id: state.id,
                name: `Robot ${state.id}`,
                state: state
            }));
            this.robots$.next(updatedRobots);
        });
    }

    public getRobots(): Observable<Robot[]> {
        return this.robots$.asObservable();
    }

    public identifyRobot(robotId: string): void {
        this.wsService.sendMessage('IDENTIFY_ROBOT', { robotId });
    }

    public startMission(): void {
        this.wsService.sendMessage('START_MISSION');
    }

    public stopMission(): void {
        this.wsService.sendMessage('STOP_MISSION');
    }

    public returnToBase(): void {
        this.wsService.sendMessage('RETURN_TO_BASE');
    }

    public setWheelMode(robotId: string, mode: WheelMode): void {
        this.wsService.sendMessage('SET_WHEEL_MODE', { robotId, mode });
    }

    public enableP2P(): void {
        this.wsService.sendMessage('ENABLE_P2P');
    }

    public disableP2P(): void {
        this.wsService.sendMessage('DISABLE_P2P');
    }

    public getRobotStatus(robotId: string): Observable<RobotState | undefined> {
        return this.robots$.pipe(
            map(robots => robots.find(robot => robot.id === robotId)?.state)
        );
    }

    public getFarthestRobot(): Observable<Robot | undefined> {
        return this.robots$.pipe(
            map(robots => robots.find(robot => robot.state.isFarthest))
        );
    }
}