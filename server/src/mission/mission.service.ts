import { Injectable, Logger } from '@nestjs/common';
import { Mission } from '../interfaces/mission.interface';
import { v4 as uuidv4 } from 'uuid';
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class MissionService {
  private readonly logger = new Logger(MissionService.name);
  private missions: Map<string, Mission> = new Map();
  private currentMission: Mission | null = null;
  private node: any;
  private publisher: any;
  private initPromise: Promise<void>;

  constructor() { // le bon = 192.168
    this.initPromise = rclnodejs.init()
      .then(() => {
        this.node = rclnodejs.createNode('mission_service_node');
        this.publisher = this.node.createPublisher('std_msgs/msg/String', '/messages');
        rclnodejs.spin(this.node);
        this.logger.log('rclnodejs initialized and node is spinning');
      })
      .catch((err) => {
        this.logger.error('Failed to initialize rclnodejs', err);
      });
  }

  async startMission(): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log('start mission');

    const message = {
      data: JSON.stringify({ action: 'start_mission' }),
    };

    this.publisher.publish(message);
    this.logger.log('Published start_mission message');

    return { message: 'Mission started' };
  }

  async stopMission(): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log('end mission');

    const message = {
      data: JSON.stringify({ action: 'end_mission' }),
    };

    this.publisher.publish(message);
    this.logger.log('Published end_mission message');

    return { message: 'Mission ended' };
  }

  async identify(): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log('Sending identify signal');

    const identifyPublisher = this.node.createPublisher('std_msgs/msg/Empty', '/identify');
    identifyPublisher.publish({});
    this.logger.log('Published identify message');

    return { message: 'Identification signal sent' };
  }
}