import { NestFactory } from '@nestjs/core';
import { AppModule } from './app.module';

async function bootstrap() {
  const app = await NestFactory.create(AppModule, {
    logger: ['error', 'warn', 'log', 'debug', 'verbose'], // Activer tous les niveaux de logs
    cors: {
      origin: ['http://localhost:4200'], // Autoriser l'origine du client Angular
      methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
      credentials: true,
      allowedHeaders: ['Content-Type', 'Authorization'],
    },
  });

  // Middleware de logging
  app.use((req, res, next) => {
    console.log(`[${new Date().toISOString()}] ${req.method} ${req.url}`);
    next();
  });

  // app.setGlobalPrefix('api'); // Préfixe désactivé

  await app.listen(3000);
  console.log('Application is running on: http://localhost:3000');
}
bootstrap();
