/**
 * Authentication Routes
 * 
 * Implements authentication endpoints per contracts/auth.openapi.yml
 * 
 * Tasks: T036-T042
 * 
 * Endpoints:
 * - POST /api/auth/signup (T037)
 * - POST /api/auth/signin (T038)
 * - POST /api/auth/signout (T039)
 * - GET /api/auth/session (T040)
 * - GET /api/auth/oauth/:provider (T041)
 * - GET /api/auth/oauth/:provider/callback (T042)
 */

import { Router } from 'express';
import {
  authenticateUser,
  createUser,
  oauthProviders,
  signToken,
  verifyToken,
} from './config';

const router = Router();

// POST /api/auth/signup
router.post('/signup', async (req, res) => {
  try {
    const { email, password } = req.body as { email?: string; password?: string };

    if (!email || !password) {
      return res.status(400).json({ error: 'email and password are required' });
    }

    const user = await createUser(email.toLowerCase(), password);
    const token = signToken({ sub: user.id, email: user.email });

    res.status(201).json({
      user: { id: user.id, email: user.email },
      token,
    });
  } catch (err) {
    const message = err instanceof Error ? err.message : 'Signup failed';
    const status = message === 'User already exists' ? 409 : 400;
    res.status(status).json({ error: message });
  }
});

// POST /api/auth/signin
router.post('/signin', async (req, res) => {
  try {
    const { email, password } = req.body as { email?: string; password?: string };
    if (!email || !password) {
      return res.status(400).json({ error: 'email and password are required' });
    }

    const user = await authenticateUser(email.toLowerCase(), password);
    const token = signToken({ sub: user.id, email: user.email });

    res.status(200).json({
      user: { id: user.id, email: user.email },
      token,
    });
  } catch (err) {
    const message = err instanceof Error ? err.message : 'Signin failed';
    res.status(401).json({ error: message });
  }
});

// POST /api/auth/signout
router.post('/signout', async (_req, res) => {
  // Stateless JWT signout is client-driven (discard token). Provided for contract parity.
  res.status(200).json({ message: 'signed out' });
});

// GET /api/auth/session
router.get('/session', async (req, res) => {
  const authHeader = req.headers.authorization;
  const token = authHeader?.startsWith('Bearer ')
    ? authHeader.substring('Bearer '.length)
    : undefined;

  if (!token) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  try {
    const payload = verifyToken(token);
    res.status(200).json({
      token,
      user: { id: payload.sub, email: payload.email },
    });
  } catch (err) {
    const message = err instanceof Error ? err.message : 'Invalid session';
    res.status(401).json({ error: message });
  }
});

// GET /api/auth/oauth/:provider (placeholder)
router.get('/oauth/:provider', async (req, res) => {
  const { provider } = req.params;
  const config = oauthProviders[provider as keyof typeof oauthProviders];
  if (!config?.enabled) {
    return res.status(400).json({ error: `OAuth provider ${provider} not configured` });
  }
  res.status(501).json({ error: 'OAuth initiation not implemented yet' });
});

// GET /api/auth/oauth/:provider/callback (placeholder)
router.get('/oauth/:provider/callback', async (req, res) => {
  const { provider } = req.params;
  const config = oauthProviders[provider as keyof typeof oauthProviders];
  if (!config?.enabled) {
    return res.status(400).json({ error: `OAuth provider ${provider} not configured` });
  }
  res.status(501).json({ error: 'OAuth callback not implemented yet' });
});

export default router;
