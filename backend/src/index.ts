import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import dotenv from 'dotenv';
import authRouter from './modules/auth/routes';

// Load environment variables
dotenv.config();

const app = express();
const PORT = process.env.PORT || 3000;
const CORS_ORIGIN = process.env.CORS_ORIGIN || 'http://localhost:3000';

// Middleware
app.use(helmet()); // Security headers
app.use(cors({ 
  origin: [CORS_ORIGIN, 'http://localhost:3000', 'http://localhost:3001'], 
  credentials: true 
}));
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check endpoint
app.get('/health', (_req, res) => {
  res.json({
    status: 'ok',
    timestamp: new Date().toISOString(),
    env: process.env.NODE_ENV || 'development',
  });
});

// API routes will be added here as implementation progresses
app.get('/api', (_req, res) => {
  res.json({
    name: 'PDCP API',
    version: '0.1.0',
    description: 'Personalized Developer Content Platform API',
    features: {
      questionnaire: process.env.FEATURE_QUESTIONNAIRE_ENABLED === 'true',
      recommendations: process.env.FEATURE_RECOMMENDATIONS_ENABLED === 'true',
      templates: process.env.FEATURE_TEMPLATES_ENABLED === 'true',
    },
  });
});

app.use('/api/auth', authRouter);

// Error handling middleware
app.use((err: Error, _req: express.Request, res: express.Response, _next: express.NextFunction) => {
  console.error('Error:', err);
  res.status(500).json({
    error: 'Internal Server Error',
    message: process.env.NODE_ENV === 'development' ? err.message : undefined,
  });
});

// 404 handler
app.use((_req, res) => {
  res.status(404).json({ error: 'Not Found' });
});

// Start server
app.listen(PORT, () => {
  console.log(`🚀 PDCP Backend running on http://localhost:${PORT}`);
  console.log(`📝 Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log(`🔧 Feature Flags:`);
  console.log(`   Questionnaire: ${process.env.FEATURE_QUESTIONNAIRE_ENABLED === 'true'}`);
  console.log(`   Recommendations: ${process.env.FEATURE_RECOMMENDATIONS_ENABLED === 'true'}`);
  console.log(`   Templates: ${process.env.FEATURE_TEMPLATES_ENABLED === 'true'}`);
});

export default app;
