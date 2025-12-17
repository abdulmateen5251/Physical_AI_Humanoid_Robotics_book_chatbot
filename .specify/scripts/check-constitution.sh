#!/bin/bash
# Constitution Compliance Check Script
# Validates implementation against constitution principles

set -e

CONSTITUTION_FILE=".specify/memory/constitution.md"
SPEC_DIR="specs/"
EXIT_CODE=0

echo "=== Constitution Compliance Check ==="
echo ""

# Check if constitution exists
if [ ! -f "$CONSTITUTION_FILE" ]; then
    echo "❌ ERROR: Constitution file not found at $CONSTITUTION_FILE"
    exit 1
fi

echo "✅ Constitution file found"
echo ""

# Principle 1: Spec-Driven Development
echo "📋 Principle 1: Spec-Driven Development"
SPEC_COUNT=$(find "$SPEC_DIR" -name "spec.md" 2>/dev/null | wc -l)
PLAN_COUNT=$(find "$SPEC_DIR" -name "plan.md" 2>/dev/null | wc -l)
TASKS_COUNT=$(find "$SPEC_DIR" -name "tasks.md" 2>/dev/null | wc -l)

echo "   Specifications: $SPEC_COUNT"
echo "   Plans: $PLAN_COUNT"
echo "   Tasks: $TASKS_COUNT"

if [ "$SPEC_COUNT" -eq "$PLAN_COUNT" ] && [ "$SPEC_COUNT" -eq "$TASKS_COUNT" ]; then
    echo "   ✅ All specs have corresponding plans and tasks"
else
    echo "   ⚠️  WARNING: Spec/plan/tasks count mismatch"
    EXIT_CODE=1
fi
echo ""

# Principle 2: Privacy & Consent by Design
echo "🔒 Principle 2: Privacy & Consent by Design"
if grep -r "optIn\|opt-in\|consent" "$SPEC_DIR" >/dev/null 2>&1; then
    echo "   ✅ Privacy controls referenced in specs"
else
    echo "   ⚠️  WARNING: No privacy controls found in specs"
    EXIT_CODE=1
fi
echo ""

# Principle 3: Security First
echo "🛡️  Principle 3: Security First"
if [ -f ".github/workflows/ci.yml" ]; then
    echo "   ✅ CI pipeline exists"
else
    echo "   ❌ ERROR: No CI pipeline found"
    EXIT_CODE=1
fi

if [ -f ".gitignore" ] && grep -q "\.env" .gitignore; then
    echo "   ✅ Environment files properly ignored"
else
    echo "   ⚠️  WARNING: .env files may not be properly ignored"
fi
echo ""

# Principle 4: Accessibility & Inclusivity
echo "♿ Principle 4: Accessibility & Inclusivity"
if grep -r "WCAG\|a11y\|accessibility" "$SPEC_DIR" >/dev/null 2>&1; then
    echo "   ✅ Accessibility requirements mentioned in specs"
else
    echo "   ⚠️  WARNING: No accessibility requirements found"
    EXIT_CODE=1
fi
echo ""

# Principle 5: Observability & Learning
echo "📊 Principle 5: Observability & Learning"
if grep -r "metrics\|analytics\|observability" "$SPEC_DIR" >/dev/null 2>&1; then
    echo "   ✅ Observability requirements found"
else
    echo "   ⚠️  WARNING: No observability requirements found"
    EXIT_CODE=1
fi
echo ""

# Principle 6: Automation & Quality
echo "🤖 Principle 6: Automation & Quality"
if [ -f ".github/workflows/ci.yml" ]; then
    if grep -q "lint\|test" .github/workflows/ci.yml; then
        echo "   ✅ Automated testing in CI"
    else
        echo "   ⚠️  WARNING: CI exists but may lack linting/testing"
        EXIT_CODE=1
    fi
else
    echo "   ❌ ERROR: No CI automation found"
    EXIT_CODE=1
fi
echo ""

# Summary
echo "=== Summary ==="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ All constitution principles satisfied"
else
    echo "⚠️  Some principles need attention (see warnings above)"
fi

exit $EXIT_CODE
