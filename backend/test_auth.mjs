/**
 * Authentication Test Script
 * Tests signup, signin, and session endpoints
 */

const API_BASE = 'http://localhost:3000';

// Helper function to make requests
async function request(method, path, body = null, token = null) {
  const headers = {
    'Content-Type': 'application/json',
  };
  
  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const options = {
    method,
    headers,
  };

  if (body) {
    options.body = JSON.stringify(body);
  }

  const response = await fetch(`${API_BASE}${path}`, options);
  const data = await response.json();
  
  return {
    status: response.status,
    ok: response.ok,
    data,
  };
}

async function runTests() {
  console.log('🧪 Starting Authentication Tests\n');

  const testEmail = `test-${Date.now()}@example.com`;
  const testPassword = 'Test123!@#';
  let authToken = null;

  // Test 1: Signup
  console.log('1️⃣ Testing Signup...');
  try {
    const signupResult = await request('POST', '/api/auth/signup', {
      email: testEmail,
      password: testPassword,
    });

    if (signupResult.status === 201 && signupResult.data.token) {
      console.log('✅ Signup successful');
      console.log(`   User ID: ${signupResult.data.user.id}`);
      console.log(`   Email: ${signupResult.data.user.email}`);
      authToken = signupResult.data.token;
    } else {
      console.log('❌ Signup failed:', signupResult.data);
      return;
    }
  } catch (error) {
    console.log('❌ Signup error:', error.message);
    return;
  }

  console.log('');

  // Test 2: Signin with correct credentials
  console.log('2️⃣ Testing Signin (correct credentials)...');
  try {
    const signinResult = await request('POST', '/api/auth/signin', {
      email: testEmail,
      password: testPassword,
    });

    if (signinResult.status === 200 && signinResult.data.token) {
      console.log('✅ Signin successful');
      console.log(`   User ID: ${signinResult.data.user.id}`);
      authToken = signinResult.data.token;
    } else {
      console.log('❌ Signin failed:', signinResult.data);
    }
  } catch (error) {
    console.log('❌ Signin error:', error.message);
  }

  console.log('');

  // Test 3: Signin with wrong password
  console.log('3️⃣ Testing Signin (wrong password)...');
  try {
    const wrongSigninResult = await request('POST', '/api/auth/signin', {
      email: testEmail,
      password: 'WrongPassword123!',
    });

    if (wrongSigninResult.status === 401) {
      console.log('✅ Correctly rejected wrong password');
    } else {
      console.log('❌ Should have rejected wrong password:', wrongSigninResult.data);
    }
  } catch (error) {
    console.log('❌ Error:', error.message);
  }

  console.log('');

  // Test 4: Session verification
  console.log('4️⃣ Testing Session Verification...');
  try {
    const sessionResult = await request('GET', '/api/auth/session', null, authToken);

    if (sessionResult.status === 200 && sessionResult.data.user) {
      console.log('✅ Session verified');
      console.log(`   User ID: ${sessionResult.data.user.id}`);
      console.log(`   Email: ${sessionResult.data.user.email}`);
    } else {
      console.log('❌ Session verification failed:', sessionResult.data);
    }
  } catch (error) {
    console.log('❌ Session error:', error.message);
  }

  console.log('');

  // Test 5: Session without token
  console.log('5️⃣ Testing Session (no token)...');
  try {
    const noTokenResult = await request('GET', '/api/auth/session');

    if (noTokenResult.status === 401) {
      console.log('✅ Correctly rejected request without token');
    } else {
      console.log('❌ Should have rejected request without token:', noTokenResult.data);
    }
  } catch (error) {
    console.log('❌ Error:', error.message);
  }

  console.log('');

  // Test 6: Signout
  console.log('6️⃣ Testing Signout...');
  try {
    const signoutResult = await request('POST', '/api/auth/signout', null, authToken);

    if (signoutResult.status === 200) {
      console.log('✅ Signout successful');
    } else {
      console.log('❌ Signout failed:', signoutResult.data);
    }
  } catch (error) {
    console.log('❌ Signout error:', error.message);
  }

  console.log('\n✨ All tests completed!\n');
}

// Run tests
runTests().catch(console.error);
