/**
 * Authentication configuration and helpers.
 *
 * NOTE: We are using bcrypt + JWT to provide working authentication flows while
 * Better Auth integration is finalized. ENV-gated providers can be wired later
 * without changing the route contracts.
 *
 * Tasks: T032-T035 (email/password now, OAuth placeholders for later)
 * Reference: contracts/auth.openapi.yml, research.md R1
 */

import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import { prisma } from '../../lib/prisma';

const AUTH_SECRET = process.env.AUTH_SECRET;
if (!AUTH_SECRET) {
  throw new Error('AUTH_SECRET is required for authentication. Set it in .env');
}

const JWT_EXPIRES_IN = process.env.JWT_EXPIRES_IN || '7d';
const JWT_ISSUER = process.env.JWT_ISSUER || 'pdcp-auth';

export type AuthTokenPayload = {
  sub: string; // userId
  email: string;
};

export async function hashPassword(password: string): Promise<string> {
  const saltRounds = 10;
  return bcrypt.hash(password, saltRounds);
}

export async function verifyPassword(password: string, hash: string): Promise<boolean> {
  return bcrypt.compare(password, hash);
}

export function signToken(payload: AuthTokenPayload): string {
  return jwt.sign(payload, AUTH_SECRET!, {
    expiresIn: JWT_EXPIRES_IN,
    issuer: JWT_ISSUER,
  });
}

export function verifyToken(token: string): AuthTokenPayload {
  try {
    const decoded = jwt.verify(token, AUTH_SECRET!, { issuer: JWT_ISSUER }) as unknown;
    
    if (!decoded || typeof decoded !== 'object' || !('email' in decoded) || !('sub' in decoded)) {
      throw new Error('Invalid token payload');
    }
    
    return decoded as AuthTokenPayload;
  } catch (error) {
    if (error instanceof jwt.JsonWebTokenError) {
      throw new Error('Invalid or expired token');
    }
    throw error;
  }
}

export async function createUser(email: string, password: string) {
  const existing = await prisma.user.findUnique({ where: { email } });
  if (existing) {
    throw new Error('User already exists');
  }

  const passwordHash = await hashPassword(password);

  const user = await prisma.user.create({
    data: {
      email,
      passwordHash,
    },
  });

  return user;
}

export async function authenticateUser(email: string, password: string) {
  const user = await prisma.user.findUnique({ where: { email } });
  if (!user) {
    throw new Error('Invalid credentials');
  }

  const passwordHash: string | undefined = user.passwordHash;

  if (!passwordHash) {
    throw new Error('Password not set for this user');
  }

  const valid = await verifyPassword(password, passwordHash);
  if (!valid) {
    throw new Error('Invalid credentials');
  }

  return user;
}

// Placeholder for future OAuth provider wiring
export const oauthProviders = {
  github: {
    enabled: Boolean(process.env.GITHUB_CLIENT_ID && process.env.GITHUB_CLIENT_SECRET),
  },
  google: {
    enabled: Boolean(process.env.GOOGLE_CLIENT_ID && process.env.GOOGLE_CLIENT_SECRET),
  },
};
