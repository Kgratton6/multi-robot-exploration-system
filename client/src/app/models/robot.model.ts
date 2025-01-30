export interface Position {
    x: number;
    y: number;
}

export interface WheelMode {
    type: 'ackerman' | 'differential';
}

export interface RobotState {
    id: string;
    batteryLevel: number;
    position: Position;
    status: 'waiting' | 'on_mission' | 'returning' | 'charging';
    wheelMode: WheelMode;
    isP2PEnabled: boolean;
    isFarthest: boolean;
}

export interface Robot {
    id: string;
    name: string;
    state: RobotState;
}