import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { BehaviorSubject, Observable } from 'rxjs';
import { environment } from '../../environments/environment';
import { Mission, MissionFilter, MissionLog, MapData } from '../models/mission.model';
import { WebSocketService } from './websocket.service';

@Injectable({
    providedIn: 'root'
})
export class MissionService {
    private currentMission$ = new BehaviorSubject<Mission | null>(null);
    private missionHistory$ = new BehaviorSubject<Mission[]>([]);

    constructor(
        private http: HttpClient,
        private wsService: WebSocketService
    ) {
        // Subscribe to mission logs from WebSocket
        this.wsService.getMissionLogs().subscribe(log => {
            this.addLogToCurrentMission(log);
        });
    }

    private addLogToCurrentMission(log: MissionLog): void {
        const currentMission = this.currentMission$.value;
        if (currentMission) {
            currentMission.logs = [...currentMission.logs, log];
            this.currentMission$.next(currentMission);
        }
    }

    private createParams(filter?: MissionFilter): HttpParams {
        let params = new HttpParams();
        if (filter) {
            if (filter.startDate) {
                params = params.set('startDate', filter.startDate.toISOString());
            }
            if (filter.endDate) {
                params = params.set('endDate', filter.endDate.toISOString());
            }
            if (filter.robotId) {
                params = params.set('robotId', filter.robotId);
            }
            if (filter.status) {
                params = params.set('status', filter.status);
            }
        }
        return params;
    }

    public getCurrentMission(): Observable<Mission | null> {
        return this.currentMission$.asObservable();
    }

    public getMissionHistory(): Observable<Mission[]> {
        return this.missionHistory$.asObservable();
    }

    public loadMissionHistory(filter?: MissionFilter): Observable<Mission[]> {
        return this.http.get<Mission[]>(`${environment.apiUrl}/missions`, {
            params: this.createParams(filter)
        });
    }

    public getMissionById(id: string): Observable<Mission> {
        return this.http.get<Mission>(`${environment.apiUrl}/missions/${id}`);
    }

    public saveMap(missionId: string, mapData: MapData): Observable<void> {
        return this.http.post<void>(
            `${environment.apiUrl}/missions/${missionId}/map`,
            mapData
        );
    }

    public loadMap(missionId: string): Observable<MapData> {
        return this.http.get<MapData>(
            `${environment.apiUrl}/missions/${missionId}/map`
        );
    }

    public downloadLogs(missionId: string): Observable<MissionLog[]> {
        return this.http.get<MissionLog[]>(
            `${environment.apiUrl}/missions/${missionId}/logs`
        );
    }

    public filterMissions(filter: MissionFilter): Observable<Mission[]> {
        return this.http.get<Mission[]>(`${environment.apiUrl}/missions`, {
            params: this.createParams(filter)
        });
    }

    public startNewMission(): void {
        const newMission: Mission = {
            id: Date.now().toString(), // Temporary ID, will be replaced by server
            startTime: new Date().toISOString(),
            status: 'ongoing',
            robots: [],
            totalDistance: 0,
            logs: []
        };
        this.currentMission$.next(newMission);
    }

    public endCurrentMission(): void {
        const mission = this.currentMission$.value;
        if (mission) {
            mission.endTime = new Date().toISOString();
            mission.status = 'completed';
            mission.duration = this.calculateDuration(mission.startTime, mission.endTime);
            
            // Update mission history
            const history = this.missionHistory$.value;
            this.missionHistory$.next([...history, mission]);
            
            // Clear current mission
            this.currentMission$.next(null);
        }
    }

    private calculateDuration(startTime: string, endTime: string): number {
        return (new Date(endTime).getTime() - new Date(startTime).getTime()) / 1000;
    }
}