import { Injectable } from '@angular/core';
import { webSocket, WebSocketSubject } from 'rxjs/webSocket';
import { BehaviorSubject, Observable, Subject } from 'rxjs';
import { environment } from '../../environments/environment';
import { RobotState } from '../models/robot.model';
import { MissionLog } from '../models/mission.model';

export interface WebSocketMessage {
    type: string;
    payload: any;
}

@Injectable({
    providedIn: 'root'
})
export class WebSocketService {
    private socket$!: WebSocketSubject<WebSocketMessage>;
    private connected$ = new BehaviorSubject<boolean>(false);
    private robotStates$ = new BehaviorSubject<RobotState[]>([]);
    private missionLogs$ = new Subject<MissionLog>();

    constructor() {
        this.connect();
    }

    private connect() {
        if (!this.socket$ || this.socket$.closed) {
            this.socket$ = webSocket({
                url: `${environment.wsUrl}`,
                openObserver: {
                    next: () => {
                        console.log('WebSocket connected');
                        this.connected$.next(true);
                    }
                },
                closeObserver: {
                    next: () => {
                        console.log('WebSocket disconnected');
                        this.connected$.next(false);
                        // Attempt to reconnect after 3 seconds
                        setTimeout(() => this.connect(), 3000);
                    }
                }
            });

            this.socket$.subscribe({
                next: (message: WebSocketMessage) => this.handleMessage(message),
                error: (error) => {
                    console.error('WebSocket error:', error);
                    this.connected$.next(false);
                }
            });
        }
    }

    private handleMessage(message: WebSocketMessage) {
        switch (message.type) {
            case 'ROBOT_STATES':
                this.robotStates$.next(message.payload);
                break;
            case 'MISSION_LOG':
                this.missionLogs$.next(message.payload);
                break;
            default:
                console.warn('Unknown message type:', message.type);
        }
    }

    public sendMessage(type: string, payload: any = {}) {
        if (this.connected$.value) {
            this.socket$.next({ type, payload });
        } else {
            console.warn('WebSocket not connected. Message not sent:', { type, payload });
        }
    }

    public getRobotStates(): Observable<RobotState[]> {
        return this.robotStates$.asObservable();
    }

    public getMissionLogs(): Observable<MissionLog> {
        return this.missionLogs$.asObservable();
    }

    public isConnected(): Observable<boolean> {
        return this.connected$.asObservable();
    }

    public disconnect() {
        if (this.socket$) {
            this.socket$.complete();
        }
    }
}