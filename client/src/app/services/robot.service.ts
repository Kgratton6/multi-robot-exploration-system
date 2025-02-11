import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { BehaviorSubject, Observable } from 'rxjs';
import { RobotState } from '../models/robot.model';
import { environment } from '../../environments/environment';

@Injectable({
  providedIn: 'root'
})
export class RobotService {
  private robotsSubject = new BehaviorSubject<RobotState[]>([]);
  private apiUrl = `${environment.apiUrl}/robots`;

  constructor(private http: HttpClient) {
    this.fetchRobotStates();
  }

  private fetchRobotStates(): void {
    this.http.get<RobotState[]>(`${this.apiUrl}/states`).subscribe(states => {
      this.robotsSubject.next(states);
    });
  }

  getRobots(): Observable<RobotState[]> {
    return this.robotsSubject.asObservable();
  }

  identifyRobot(robotId: string): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/${robotId}/identify`, {});
  }

  startMission(robotIds: string[]): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/mission/start`, { robotIds });
  }

  stopMission(robotIds: string[]): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/mission/stop`, { robotIds });
  }

  returnToBase(robotIds: string[]): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/mission/return`, { robotIds });
  }

  setWheelMode(robotId: string, mode: 'ackerman' | 'differential'): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/${robotId}/wheel-mode`, { mode });
  }

  enableP2P(robotIds: string[]): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/p2p/enable`, { robotIds });
  }

  disableP2P(robotIds: string[]): Observable<void> {
    return this.http.post<void>(`${this.apiUrl}/p2p/disable`, { robotIds });
  }
}