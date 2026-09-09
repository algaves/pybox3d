## Description
<!-- Provide a brief summary of the Python changes and the rationale behind them. -->

## Related Issues
<!-- Link to the issue(s) this PR fixes (e.g., Closes #123). -->

## Type of Change
- [ ] ✨ New feature (non-breaking change which adds functionality)
- [ ] 🐛 Bug fix (non-breaking change which fixes an issue)
- [ ] ♻️ Refactoring / Performance improvement
- [ ] 📝 Documentation update (Docstrings, README, MkDocs)
- [ ] 🛠️ C-extension build improvement

## Python Quality Checklist
- [ ] **Formatting:** Code has been formatted using `black`, `ruff`, or `yapf`.
- [ ] **Linting:** Code passes `flake8`, `pylint`, or `ruff check` without errors.
- [ ] **Type Hints:** Type checking passes using `mypy` (if applicable).
- [ ] **Dependencies:** Any new packages have been added to `pyproject.toml`.
- [ ] **CMake Build:** Native extension compiles without warnings.

## Testing Environment
- [ ] **Unit Tests:** Run locally via `pytest tests/`
- [ ] **Test Coverage:** Code coverage has met the project threshold.
- [ ] **C-extension Tests:** Native module tests pass (if applicable).

### Test Command Used:
```bash
pytest tests/ --cov=src/
```

## Impact Assessment
- **API Changes:** [breaking / non-breaking]
- **Performance:** [expected impact]
- **Dependencies:** [new packages or version bumps]

## Alternative Considered
<!-- Describe any alternative solutions or features you considered -->

---
**Template reference:** This request uses the standardized templates in `.github/`. See `template_request.md` for requesting new template types.
