import { PrismaClient } from '@prisma/client';

const prisma = new PrismaClient();

async function main() {
  console.log('🌱 Seeding database...');

  // Seed Content Items
  console.log('Creating content items...');
  const contentItems = await Promise.all([
    prisma.contentItem.create({
      data: {
        title: 'Getting Started with Next.js 14',
        description: 'Comprehensive guide to building modern web applications with Next.js 14, covering App Router, Server Components, and streaming.',
        contentUrl: 'https://example.com/nextjs-guide',
        tags: JSON.stringify({
          technologies: ['TypeScript', 'Next.js', 'React'],
          difficulty: 'intermediate',
          category: 'web-development',
        }),
      },
    }),
    prisma.contentItem.create({
      data: {
        title: 'Python Machine Learning Fundamentals',
        description: 'Learn the basics of machine learning with Python, including scikit-learn, pandas, and NumPy for data analysis and model building.',
        contentUrl: 'https://example.com/python-ml',
        tags: JSON.stringify({
          technologies: ['Python', 'scikit-learn', 'pandas'],
          difficulty: 'beginner',
          category: 'machine-learning',
        }),
      },
    }),
    prisma.contentItem.create({
      data: {
        title: 'Building RESTful APIs with FastAPI',
        description: 'Modern Python API development with FastAPI, covering async operations, validation, and automatic API documentation.',
        contentUrl: 'https://example.com/fastapi-guide',
        tags: JSON.stringify({
          technologies: ['Python', 'FastAPI', 'PostgreSQL'],
          difficulty: 'intermediate',
          category: 'backend-development',
        }),
      },
    }),
    prisma.contentItem.create({
      data: {
        title: 'Rust for Systems Programming',
        description: 'Deep dive into Rust programming language for building reliable and efficient systems software with memory safety.',
        contentUrl: 'https://example.com/rust-systems',
        tags: JSON.stringify({
          technologies: ['Rust'],
          difficulty: 'advanced',
          category: 'systems-programming',
        }),
      },
    }),
    prisma.contentItem.create({
      data: {
        title: 'Docker and Containerization Basics',
        description: 'Learn containerization with Docker, from creating Dockerfiles to orchestrating multi-container applications.',
        contentUrl: 'https://example.com/docker-basics',
        tags: JSON.stringify({
          technologies: ['Docker'],
          difficulty: 'beginner',
          category: 'devops',
        }),
      },
    }),
  ]);

  console.log(`✅ Created ${contentItems.length} content items`);

  // Seed Starter Templates
  console.log('Creating starter templates...');
  const templates = await Promise.all([
    prisma.starterTemplate.create({
      data: {
        name: 'Next.js + TypeScript Starter',
        description: 'Full-stack Next.js 14 template with TypeScript, Tailwind CSS, and ESLint configured.',
        supportedTechnologies: JSON.stringify({
          languages: ['TypeScript'],
          frameworks: ['Next.js', 'React'],
          os: ['Windows', 'macOS', 'Linux'],
        }),
        templateFiles: 'https://github.com/example/nextjs-ts-starter',
      },
    }),
    prisma.starterTemplate.create({
      data: {
        name: 'Python + FastAPI Backend',
        description: 'RESTful API template with FastAPI, PostgreSQL, Prisma, and Docker configuration.',
        supportedTechnologies: JSON.stringify({
          languages: ['Python'],
          frameworks: ['FastAPI'],
          os: ['Windows', 'macOS', 'Linux'],
        }),
        templateFiles: 'https://github.com/example/fastapi-starter',
      },
    }),
    prisma.starterTemplate.create({
      data: {
        name: 'Rust CLI Application',
        description: 'Command-line application template with Rust, clap for argument parsing, and error handling.',
        supportedTechnologies: JSON.stringify({
          languages: ['Rust'],
          frameworks: [],
          os: ['Windows', 'macOS', 'Linux'],
        }),
        templateFiles: 'https://github.com/example/rust-cli-starter',
      },
    }),
    prisma.starterTemplate.create({
      data: {
        name: 'Python + Django Web App',
        description: 'Full-stack Django template with authentication, admin panel, and PostgreSQL database.',
        supportedTechnologies: JSON.stringify({
          languages: ['Python'],
          frameworks: ['Django'],
          os: ['Windows', 'macOS', 'Linux'],
        }),
        templateFiles: 'https://github.com/example/django-starter',
      },
    }),
    prisma.starterTemplate.create({
      data: {
        name: 'Node.js + Express API',
        description: 'Minimal Express.js API with TypeScript, JWT authentication, and Prisma ORM.',
        supportedTechnologies: JSON.stringify({
          languages: ['TypeScript', 'JavaScript'],
          frameworks: ['Express'],
          os: ['Windows', 'macOS', 'Linux'],
        }),
        templateFiles: 'https://github.com/example/express-ts-starter',
      },
    }),
  ]);

  console.log(`✅ Created ${templates.length} starter templates`);

  console.log('');
  console.log('🎉 Seeding complete!');
  console.log(`   Content Items: ${contentItems.length}`);
  console.log(`   Starter Templates: ${templates.length}`);
}

main()
  .catch((e) => {
    console.error('❌ Error seeding database:', e);
    process.exit(1);
  })
  .finally(async () => {
    await prisma.$disconnect();
  });
