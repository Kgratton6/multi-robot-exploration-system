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

  constructor() {
    this.initPromise = rclnodejs.init()
      .then(() => {
        this.node = rclnodejs.createNode('mission_service_node');
        this.publisher = this.node.createPublisher('std_msgs/msg/String', '/messages');
        rclnodejs.spin(this.node);
        this.logger.log('rclnodejs initialisé et node en exécution');
      })
      .catch((err) => {
        this.logger.error('Échec de l’init de rclnodejs', err);
      });
  }

  async startMission(robotId: string): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log(`Démarrage de la mission pour ${robotId}`);
    const message = {
      data: JSON.stringify({ action: 'start_mission', robot_id: robotId }),
    };
    this.publisher.publish(message);
    this.logger.log('Message start_mission publié');
    return { message: `Mission démarrée pour ${robotId}` };
  }

  async stopMission(robotId: string): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log(`Arrêt de la mission pour ${robotId}`);
    const message = {
      data: JSON.stringify({ action: 'end_mission', robot_id: robotId }),
    };
    this.publisher.publish(message);
    this.logger.log('Message end_mission publié');
    return { message: `Mission arrêtée pour ${robotId}` };
  }

  async identify(robotId: string): Promise<{ message: string }> {
    await this.initPromise;
    this.logger.log(`Envoi du signal d’identification pour ${robotId}`);
    const identifyTopic = `/${robotId}/identify`;
    const identifyPublisher = this.node.createPublisher('std_msgs/msg/Empty', identifyTopic);
    identifyPublisher.publish({});
    this.logger.log(`Message d’identification publié pour ${robotId}`);
    return { message: `Signal d’identification envoyé pour ${robotId}` };
  }
}
