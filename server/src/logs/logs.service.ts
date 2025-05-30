import { Injectable, NotFoundException, Logger } from '@nestjs/common';
import { InjectModel } from '@nestjs/mongoose';
import { Model } from 'mongoose';
import { MissionLog, MissionLogDocument } from './schemas/mission-log.schema';
import { Server } from 'socket.io';


@Injectable()
export class LogsService {
  server: Server;
  private readonly logger = new Logger(LogsService.name);
  private activeLogCache: Map<string, MissionLogDocument> = new Map();

  constructor(
    @InjectModel(MissionLog.name) private readonly missionLogModel: Model<MissionLogDocument>
  ) {
    this.logger.log('LogsService initialized');
  }
  initialize(server: Server) {
    this.server = server;
  }

  async initializeMissionLog(missionId: string): Promise<MissionLogDocument> {
    this.logger.debug(`Initializing log for mission: ${missionId}`);
    
    const missionLog = new this.missionLogModel({
      missionId,
      startTime: new Date(),
      logs: []
    });
    
    try {
      const savedLog = await missionLog.save();
      this.activeLogCache.set(missionId, savedLog);
      this.logger.debug(`✅ Mission log initialized with ID: ${savedLog._id}`);
      return savedLog;
    } catch (error) {
      this.logger.error(`❌ Error initializing mission log: ${error.message}`, error.stack);
      throw error;
    }
  }

  async addLog(missionId: string, logEntry: { type: 'SENSOR' | 'COMMAND', robotId: string, data: any }) {
    this.logger.debug(`Adding log to mission ${missionId}: ${JSON.stringify(logEntry)}`);
    
    try {
      const updatedLog = await this.missionLogModel.findOneAndUpdate(
        { missionId },
        {
          $push: {
            logs: {
              timestamp: new Date(),
              ...logEntry
            }
          }
        },
        { new: true, upsert: true }
      );

      this.activeLogCache.set(missionId, updatedLog);
      this.logger.debug(`✅ Log entry added to mission ${missionId}`);
      const missionLog = await this.findMissionLog(missionId);
      this.server.emit('missionLogs', missionLog.logs);
    } catch (error) {
      this.logger.error(`❌ Error adding log entry: ${error.message}`, error.stack);
      throw error;
    }
  }

  async finalizeMissionLog(missionId: string, totalDistance?: number) {
    this.logger.debug(`Finalizing log for mission: ${missionId}`);
    
    try {
      const updateData: any = {
        endTime: new Date()
      };

      if (totalDistance !== undefined) {
        updateData.totalDistance = totalDistance;
      }

      const updatedLog = await this.missionLogModel.findOneAndUpdate(
        { missionId },
        {
          $set: updateData
        },
        { new: true }
      );

      if (!updatedLog) {
        throw new NotFoundException(`No log found for mission ID: ${missionId}`);
      }
      
      this.activeLogCache.delete(missionId);
      this.logger.debug(`✅ Mission log finalized for ${missionId}`);
    } catch (error) {
      this.logger.error(`Error finalizing mission log: ${error.message}`, error.stack);
      throw error;
    }
  }

  async findMissionLog(missionId: string): Promise<MissionLogDocument> {
    this.logger.debug(`Finding log for mission ID: ${missionId}`);
    
    try {
      const missionLog = await this.missionLogModel.findOne({ missionId }).exec();
      
      if (!missionLog) {
        throw new NotFoundException(`No log found for mission ID: ${missionId}`);
      }
      
      return missionLog;
    } catch (error) {
      if (error instanceof NotFoundException) {
        throw error;
      }
      this.logger.error(`Error finding mission log: ${error.message}`, error.stack);
      throw error;
    }
  }

  async findAllMissionLogs(): Promise<MissionLogDocument[]> {
    this.logger.debug('Finding all mission logs');
    try {
      const logs = await this.missionLogModel.find().sort({ startTime: -1 }).exec();
      this.logger.debug(`Found ${logs.length} mission logs`);
      return logs;
    } catch (error) {
      this.logger.error(`Error finding mission logs: ${error.message}`, error.stack);
      throw error;
    }
  }

  async updateMissionMap(missionId: string, mapData: { timestamp: string; data: string }): Promise<void> {
    this.logger.debug(`Updating map for mission: ${missionId}`);
    try {
      const updatedLog = await this.missionLogModel.findOneAndUpdate(
        { missionId },
        {
          $set: {
            map: {
              timestamp: new Date(mapData.timestamp),
              data: mapData.data
            }
          }
        },
        { new: true }
      );

      if (!updatedLog) {
        throw new NotFoundException(`No log found for mission ID: ${missionId}`);
      }

      this.logger.debug(`✅ Map updated for mission ${missionId}`);
    } catch (error) {
      this.logger.error(`Error updating mission map: ${error.message}`, error.stack);
      throw error;
    }
  }
}
