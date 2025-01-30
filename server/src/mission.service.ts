//Successfully initializes rclnodejs, creates a ROS2 service, and listens for incoming mission request
import { Injectable, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class MissionService implements OnModuleInit {
  private node: rclnodejs.Node;

  async onModuleInit() {
    await rclnodejs.init();
    this.node = new rclnodejs.Node('nestjs_mission_server');

    // Créer un service ROS2
    this.node.createService(
      'std_srvs/srv/Trigger', // Type du service
      'send_mission', // Nom du service
      (request, response) => {
        console.log('Mission reçue:', request);
        response.send({ success: true, message: 'Mission acceptée!' });
      }
    );
    

    rclnodejs.spin(this.node);
    console.log('Service ROS2 "send_mission" est prêt!');
  }
}
