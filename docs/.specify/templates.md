
# Deployment Templates

This document describes the system for discovering and retrieving deployment templates.

## Template Discovery

The system discovers templates by searching for `.template.md` files in the following locations, in order of precedence:

1.  A directory specified by the `SPECIFY_TEMPLATES_DIR` environment variable.
2.  The `.specify/templates` directory in the current git repository.
3.  The user's home directory (`~/.specify/templates`).

The `discover_templates` function returns a dictionary of all discoverable templates, where the keys are the template names (without the `.template.md` suffix) and the values are their file paths.

## Template Retrieval

The `template_details` function retrieves the content and metadata of a specific template. It returns a dictionary containing the template's name, path, and content.

## Template Management

The `template_list` function returns a list of all available templates.

The `get_templates_dir` and `get_templates_path` functions are helper functions for getting the path to the templates directory and a specific template file, respectively.
