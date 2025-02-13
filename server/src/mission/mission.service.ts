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
      this.logger.log(`Démarrage de la mission pour ${robotId} sur robot1_102 et robot2_102`);
      const topics = ['/robot1_102/data', '/robot2_102/data'];
      const message = {
        data: JSON.stringify({ action: 'start_mission', robot_id: robotId }),
      };
      topics.forEach(topic => {
        const publisher = this.node.createPublisher('std_msgs/msg/String', topic);
        publisher.publish(message);
        this.logger.log(`Message start_mission publié sur ${topic}`);
      });
      return { message: `Mission démarrée pour ${robotId} sur robot1_102 et robot2_102` };
  }

  async stopMission(robotId: string): Promise<{ message: string }> {
      await this.initPromise;
      this.logger.log(`Arrêt de la mission pour ${robotId} sur robot1_102 et robot2_102`);
      const topics = ['/robot1_102/data', '/robot2_102/data'];
      const message = {
        data: JSON.stringify({ action: 'end_mission', robot_id: robotId }),
      };
      topics.forEach(topic => {
        const publisher = this.node.createPublisher('std_msgs/msg/String', topic);
        publisher.publish(message);
        this.logger.log(`Message end_mission publié sur ${topic}`);
      });
      return { message: `Mission arrêtée pour ${robotId} sur robot1_102 et robot2_102` };
  }

  async identify(robotId: string): Promise<{ message: string }> {
      await this.initPromise;
      this.logger.log(`Envoi du signal d’identification pour ${robotId}`);
      const dataTopic = `/${robotId}/data`;
      const dataPublisher = this.node.createPublisher('std_msgs/msg/String', dataTopic);
      const message = {
        data: JSON.stringify({ action: 'identify', robot_id: robotId }),
      };
      dataPublisher.publish(message);
      this.logger.log(`Message d’identification publié sur ${dataTopic}`);
      return { message: `Signal d’identification envoyé pour ${robotId}` };
  }
}
