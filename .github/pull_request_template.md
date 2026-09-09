## Pull Request Template

### Description
<!-- Provide a brief summary of the changes and the rationale behind them. -->

### Type of Change
- [ ] 🐛 Bug fix (non-breaking change which fixes an issue)
- [ ] ✨ New feature (non-breaking change which adds functionality)
- [ ] ♻️ Refactoring / Performance improvement
- [ ] 📝 Documentation update (Docstrings, README, MkDocs)
- [ ] 🛠️ C-extension build improvement
- [ ] 🎨 MkDocs theme/configuration update

### Checklist
- [ ] **Code follows project style** (black, ruff formatting)
- [ ] **Linting passes** (ruff check / pylint)
- [ ] **Type checking passes** (mypy, if applicable)
- [ ] **CMake/build compiles** without errors/warnings
- [ ] **Tests pass** (`pytest tests/`)
- [ ] **Documentation updated** (if applicable)
- [ ] **Changelog updated** (if applicable)

### Testing
<!-- Describe the tests you ran to verify your changes. Provide instructions so we can reproduce. -->

```bash
# Run the test suite
pytest tests/

# Run type checking
mypy src/

# Run linting
ruff check src/
```

### Additional Context
<!-- Describe any alternative solutions or features you considered -->

### Related Issues
<!-- Link to issue(s) this PR fixes or relates to (e.g., Closes #123) -->

### C-extension Notes (if applicable)
<!-- Any special notes about CMake, cibuildwheel, or native module changes -->
