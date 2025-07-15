# Python Project Structure Guide for Industrial Deployment

## Overview

This guide provides a comprehensive framework for transforming a large, monolithic Python program into a well-structured, maintainable package ready for industrial deployment. The structure follows industry best practices and supports scalability, testing, documentation, and deployment.

## Recommended Project Structure

```
my_package/
├── README.md
├── LICENSE
├── pyproject.toml
├── setup.py (optional, for legacy support)
├── requirements.txt
├── requirements-dev.txt
├── .gitignore
├── .env.example
├── Dockerfile
├── docker-compose.yml
├── Makefile
├── src/
│   └── my_package/
│       ├── __init__.py
│       ├── main.py
│       ├── config/
│       │   ├── __init__.py
│       │   ├── settings.py
│       │   └── logging.py
│       ├── core/
│       │   ├── __init__.py
│       │   ├── business_logic.py
│       │   ├── data_processor.py
│       │   └── algorithms.py
│       ├── models/
│       │   ├── __init__.py
│       │   ├── data_models.py
│       │   └── database_models.py
│       ├── services/
│       │   ├── __init__.py
│       │   ├── database_service.py
│       │   ├── api_service.py
│       │   └── file_service.py
│       ├── utils/
│       │   ├── __init__.py
│       │   ├── helpers.py
│       │   ├── validators.py
│       │   └── decorators.py
│       ├── cli/
│       │   ├── __init__.py
│       │   └── commands.py
│       └── exceptions/
│           ├── __init__.py
│           └── custom_exceptions.py
├── tests/
│   ├── __init__.py
│   ├── conftest.py
│   ├── unit/
│   │   ├── __init__.py
│   │   ├── test_core/
│   │   ├── test_services/
│   │   └── test_utils/
│   ├── integration/
│   │   ├── __init__.py
│   │   └── test_integration.py
│   └── fixtures/
│       ├── __init__.py
│       └── sample_data.py
├── docs/
│   ├── conf.py
│   ├── index.rst
│   ├── api/
│   ├── user_guide/
│   └── deployment/
├── scripts/
│   ├── setup.sh
│   ├── deploy.sh
│   └── migrate.py
└── data/
    ├── input/
    ├── output/
    └── processed/
```

## Step-by-Step Refactoring Process

### Phase 1: Analysis and Planning

1. **Analyze the existing codebase**
   - Identify main functions and their responsibilities
   - Map data flow and dependencies
   - Identify configuration variables and constants
   - Document external dependencies

2. **Create a dependency map**
   - List all imports and their usage
   - Identify circular dependencies
   - Plan module separation strategy

### Phase 2: Core Structure Setup

1. **Initialize the project structure**
   ```bash
   mkdir my_package
   cd my_package
   mkdir -p src/my_package/{config,core,models,services,utils,cli,exceptions}
   mkdir -p tests/{unit,integration,fixtures}
   mkdir -p docs/{api,user_guide,deployment}
   mkdir -p scripts data/{input,output,processed}
   ```

2. **Create essential configuration files**

#### pyproject.toml
```toml
[build-system]
requires = ["setuptools>=45", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "my_package"
version = "1.0.0"
description = "Industrial-ready Python package"
authors = [{name = "Your Name", email = "your.email@example.com"}]
license = {text = "MIT"}
readme = "README.md"
requires-python = ">=3.8"
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.8",
    "Programming Language :: Python :: 3.9",
    "Programming Language :: Python :: 3.10",
    "Programming Language :: Python :: 3.11",
]
dependencies = [
    "requests>=2.28.0",
    "pydantic>=1.10.0",
    "click>=8.0.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "pytest-cov>=4.0.0",
    "black>=22.0.0",
    "flake8>=5.0.0",
    "mypy>=0.991",
    "pre-commit>=2.20.0",
]
docs = [
    "sphinx>=5.0.0",
    "sphinx-rtd-theme>=1.0.0",
]

[project.scripts]
my-package = "my_package.cli:main"

[tool.setuptools.packages.find]
where = ["src"]

[tool.black]
line-length = 88
target-version = ['py38']

[tool.mypy]
python_version = "3.8"
strict = true
warn_return_any = true
warn_unused_configs = true

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "--cov=src/my_package --cov-report=html --cov-report=term-missing"
```

### Phase 3: Module Separation

#### 1. Configuration Module (`src/my_package/config/settings.py`)
```python
"""Configuration management for the application."""
import os
from typing import Optional
from pydantic import BaseSettings, Field


class Settings(BaseSettings):
    """Application settings with environment variable support."""
    
    # Application settings
    app_name: str = "My Package"
    version: str = "1.0.0"
    debug: bool = Field(False, env="DEBUG")
    
    # Database settings
    database_url: Optional[str] = Field(None, env="DATABASE_URL")
    database_timeout: int = Field(30, env="DATABASE_TIMEOUT")
    
    # API settings
    api_key: Optional[str] = Field(None, env="API_KEY")
    api_base_url: str = Field("https://api.example.com", env="API_BASE_URL")
    
    # File paths
    data_dir: str = Field("data", env="DATA_DIR")
    output_dir: str = Field("data/output", env="OUTPUT_DIR")
    
    # Performance settings
    max_workers: int = Field(4, env="MAX_WORKERS")
    batch_size: int = Field(100, env="BATCH_SIZE")
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


# Global settings instance
settings = Settings()
```

#### 2. Custom Exceptions (`src/my_package/exceptions/custom_exceptions.py`)
```python
"""Custom exceptions for the application."""


class MyPackageError(Exception):
    """Base exception for all package-related errors."""
    pass


class ConfigurationError(MyPackageError):
    """Raised when configuration is invalid."""
    pass


class DataProcessingError(MyPackageError):
    """Raised when data processing fails."""
    pass


class ValidationError(MyPackageError):
    """Raised when data validation fails."""
    pass


class ServiceError(MyPackageError):
    """Raised when external service calls fail."""
    pass
```

#### 3. Data Models (`src/my_package/models/data_models.py`)
```python
"""Data models using Pydantic for validation."""
from typing import List, Optional, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field, validator


class BaseDataModel(BaseModel):
    """Base model with common fields."""
    
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: Optional[datetime] = None
    
    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class InputData(BaseDataModel):
    """Model for input data validation."""
    
    id: str = Field(..., min_length=1, max_length=100)
    name: str = Field(..., min_length=1)
    value: float = Field(..., gt=0)
    category: str
    metadata: Dict[str, Any] = Field(default_factory=dict)
    
    @validator('category')
    def validate_category(cls, v):
        allowed_categories = ['A', 'B', 'C']
        if v not in allowed_categories:
            raise ValueError(f'Category must be one of {allowed_categories}')
        return v


class ProcessedData(BaseDataModel):
    """Model for processed data."""
    
    input_id: str
    result: float
    status: str = Field(default="completed")
    processing_time: float
    
    @validator('status')
    def validate_status(cls, v):
        allowed_statuses = ['completed', 'failed', 'pending']
        if v not in allowed_statuses:
            raise ValueError(f'Status must be one of {allowed_statuses}')
        return v
```

#### 4. Core Business Logic (`src/my_package/core/business_logic.py`)
```python
"""Core business logic separated from I/O operations."""
import logging
from typing import List, Dict, Any
from ..models.data_models import InputData, ProcessedData
from ..exceptions.custom_exceptions import DataProcessingError

logger = logging.getLogger(__name__)


class BusinessLogicProcessor:
    """Handles core business logic operations."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.logger = logger
    
    def process_single_item(self, item: InputData) -> ProcessedData:
        """Process a single data item."""
        try:
            start_time = time.time()
            
            # Core business logic here
            result = self._calculate_result(item)
            
            processing_time = time.time() - start_time
            
            return ProcessedData(
                input_id=item.id,
                result=result,
                processing_time=processing_time,
                status="completed"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to process item {item.id}: {str(e)}")
            raise DataProcessingError(f"Processing failed for item {item.id}") from e
    
    def process_batch(self, items: List[InputData]) -> List[ProcessedData]:
        """Process multiple items in batch."""
        results = []
        
        for item in items:
            try:
                result = self.process_single_item(item)
                results.append(result)
            except DataProcessingError as e:
                self.logger.warning(f"Skipping item due to error: {e}")
                continue
        
        return results
    
    def _calculate_result(self, item: InputData) -> float:
        """Internal method for core calculation."""
        # Replace with your actual business logic
        multiplier = self.config.get('multiplier', 1.0)
        return item.value * multiplier
```

#### 5. Services Layer (`src/my_package/services/database_service.py`)
```python
"""Database service for data persistence."""
import logging
from typing import List, Optional, Dict, Any
from contextlib import contextmanager
from ..models.data_models import InputData, ProcessedData
from ..exceptions.custom_exceptions import ServiceError

logger = logging.getLogger(__name__)


class DatabaseService:
    """Handles database operations."""
    
    def __init__(self, connection_string: str):
        self.connection_string = connection_string
        self.logger = logger
    
    @contextmanager
    def get_connection(self):
        """Context manager for database connections."""
        conn = None
        try:
            # Initialize your database connection here
            conn = self._create_connection()
            yield conn
        except Exception as e:
            self.logger.error(f"Database connection error: {str(e)}")
            raise ServiceError(f"Database connection failed") from e
        finally:
            if conn:
                conn.close()
    
    def save_input_data(self, data: List[InputData]) -> None:
        """Save input data to database."""
        with self.get_connection() as conn:
            # Implementation depends on your database choice
            pass
    
    def get_input_data(self, filters: Optional[Dict[str, Any]] = None) -> List[InputData]:
        """Retrieve input data from database."""
        with self.get_connection() as conn:
            # Implementation depends on your database choice
            pass
    
    def save_processed_data(self, data: List[ProcessedData]) -> None:
        """Save processed data to database."""
        with self.get_connection() as conn:
            # Implementation depends on your database choice
            pass
    
    def _create_connection(self):
        """Create database connection based on connection string."""
        # Implementation depends on your database choice
        pass
```

#### 6. Utilities (`src/my_package/utils/helpers.py`)
```python
"""Utility functions and helpers."""
import json
import csv
from typing import Any, Dict, List
from pathlib import Path
from ..exceptions.custom_exceptions import ValidationError


def load_json_file(file_path: str) -> Dict[str, Any]:
    """Load and parse JSON file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        raise ValidationError(f"Failed to load JSON file {file_path}") from e


def save_json_file(data: Dict[str, Any], file_path: str) -> None:
    """Save data to JSON file."""
    try:
        Path(file_path).parent.mkdir(parents=True, exist_ok=True)
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        raise ValidationError(f"Failed to save JSON file {file_path}") from e


def load_csv_file(file_path: str) -> List[Dict[str, Any]]:
    """Load CSV file as list of dictionaries."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            return list(reader)
    except Exception as e:
        raise ValidationError(f"Failed to load CSV file {file_path}") from e


def create_directory_if_not_exists(directory: str) -> None:
    """Create directory if it doesn't exist."""
    Path(directory).mkdir(parents=True, exist_ok=True)


def validate_file_exists(file_path: str) -> bool:
    """Check if file exists."""
    return Path(file_path).exists()
```

#### 7. CLI Interface (`src/my_package/cli/commands.py`)
```python
"""Command-line interface for the package."""
import click
import logging
from typing import Optional
from ..config.settings import settings
from ..core.business_logic import BusinessLogicProcessor
from ..services.database_service import DatabaseService
from ..utils.helpers import load_json_file, save_json_file

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@click.group()
@click.version_option()
def main():
    """My Package CLI - Industrial Python Package."""
    pass


@main.command()
@click.option('--input-file', '-i', required=True, help='Input data file path')
@click.option('--output-file', '-o', required=True, help='Output file path')
@click.option('--config-file', '-c', help='Configuration file path')
@click.option('--batch-size', '-b', default=100, help='Batch processing size')
def process(input_file: str, output_file: str, config_file: Optional[str], batch_size: int):
    """Process data from input file and save results."""
    try:
        # Load configuration
        config = {}
        if config_file:
            config = load_json_file(config_file)
        
        # Load input data
        input_data = load_json_file(input_file)
        
        # Initialize processor
        processor = BusinessLogicProcessor(config)
        
        # Process data
        # (Add your processing logic here)
        
        # Save results
        save_json_file(results, output_file)
        
        click.echo(f"Processing completed. Results saved to {output_file}")
        
    except Exception as e:
        logger.error(f"Processing failed: {str(e)}")
        click.echo(f"Error: {str(e)}", err=True)
        raise click.Abort()


@main.command()
@click.option('--database-url', help='Database connection URL')
def migrate(database_url: Optional[str]):
    """Run database migrations."""
    try:
        db_url = database_url or settings.database_url
        if not db_url:
            raise click.ClickException("Database URL is required")
        
        # Initialize database service
        db_service = DatabaseService(db_url)
        
        # Run migrations
        # (Add your migration logic here)
        
        click.echo("Migration completed successfully")
        
    except Exception as e:
        logger.error(f"Migration failed: {str(e)}")
        click.echo(f"Error: {str(e)}", err=True)
        raise click.Abort()


if __name__ == '__main__':
    main()
```

#### 8. Main Entry Point (`src/my_package/main.py`)
```python
"""Main entry point for the application."""
import logging
from typing import List, Optional
from .config.settings import settings
from .core.business_logic import BusinessLogicProcessor
from .services.database_service import DatabaseService
from .models.data_models import InputData, ProcessedData
from .exceptions.custom_exceptions import MyPackageError

# Configure logging
logging.basicConfig(
    level=logging.INFO if not settings.debug else logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class Application:
    """Main application class."""
    
    def __init__(self):
        self.settings = settings
        self.processor = BusinessLogicProcessor(self._get_processor_config())
        self.db_service = None
        if self.settings.database_url:
            self.db_service = DatabaseService(self.settings.database_url)
    
    def run(self, input_data: List[InputData]) -> List[ProcessedData]:
        """Main application execution."""
        try:
            logger.info(f"Starting processing of {len(input_data)} items")
            
            # Process data in batches
            results = []
            for i in range(0, len(input_data), self.settings.batch_size):
                batch = input_data[i:i + self.settings.batch_size]
                batch_results = self.processor.process_batch(batch)
                results.extend(batch_results)
                
                logger.info(f"Processed batch {i//self.settings.batch_size + 1}")
            
            # Save results if database is configured
            if self.db_service:
                self.db_service.save_processed_data(results)
            
            logger.info(f"Processing completed. {len(results)} items processed")
            return results
            
        except MyPackageError as e:
            logger.error(f"Application error: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error: {str(e)}")
            raise MyPackageError(f"Application failed: {str(e)}") from e
    
    def _get_processor_config(self) -> dict:
        """Get configuration for the processor."""
        return {
            'max_workers': self.settings.max_workers,
            'batch_size': self.settings.batch_size,
            'multiplier': 1.5,  # Example configuration
        }


def main():
    """Main function for direct execution."""
    app = Application()
    
    # Example usage
    sample_data = [
        InputData(id="1", name="Sample 1", value=10.0, category="A"),
        InputData(id="2", name="Sample 2", value=20.0, category="B"),
    ]
    
    results = app.run(sample_data)
    
    for result in results:
        print(f"Processed {result.input_id}: {result.result}")


if __name__ == '__main__':
    main()
```

### Phase 4: Testing Strategy

#### Test Configuration (`tests/conftest.py`)
```python
"""Pytest configuration and fixtures."""
import pytest
from unittest.mock import Mock
from src.my_package.config.settings import Settings
from src.my_package.models.data_models import InputData


@pytest.fixture
def mock_settings():
    """Mock settings for testing."""
    return Settings(
        debug=True,
        database_url="sqlite:///:memory:",
        batch_size=10
    )


@pytest.fixture
def sample_input_data():
    """Sample input data for testing."""
    return [
        InputData(id="1", name="Test 1", value=10.0, category="A"),
        InputData(id="2", name="Test 2", value=20.0, category="B"),
    ]


@pytest.fixture
def mock_database_service():
    """Mock database service."""
    mock = Mock()
    mock.save_processed_data.return_value = None
    return mock
```

#### Unit Test Example (`tests/unit/test_core/test_business_logic.py`)
```python
"""Unit tests for business logic."""
import pytest
from src.my_package.core.business_logic import BusinessLogicProcessor
from src.my_package.models.data_models import InputData
from src.my_package.exceptions.custom_exceptions import DataProcessingError


class TestBusinessLogicProcessor:
    """Test cases for BusinessLogicProcessor."""
    
    def test_process_single_item_success(self, sample_input_data):
        """Test successful processing of single item."""
        processor = BusinessLogicProcessor({'multiplier': 2.0})
        item = sample_input_data[0]
        
        result = processor.process_single_item(item)
        
        assert result.input_id == item.id
        assert result.result == item.value * 2.0
        assert result.status == "completed"
    
    def test_process_batch_success(self, sample_input_data):
        """Test successful batch processing."""
        processor = BusinessLogicProcessor({'multiplier': 1.5})
        
        results = processor.process_batch(sample_input_data)
        
        assert len(results) == 2
        assert all(r.status == "completed" for r in results)
    
    def test_process_single_item_failure(self):
        """Test handling of processing failure."""
        processor = BusinessLogicProcessor({})
        # Create invalid input that will cause processing to fail
        invalid_item = InputData(id="invalid", name="Invalid", value=-1.0, category="A")
        
        with pytest.raises(DataProcessingError):
            processor.process_single_item(invalid_item)
```

### Phase 5: Documentation

#### README.md Template
```markdown
# My Package

Industrial-ready Python package for [brief description].

## Features

- Modular architecture with clear separation of concerns
- Comprehensive error handling and logging
- Type hints and data validation with Pydantic
- Command-line interface
- Extensive test coverage
- Docker support for easy deployment

## Installation

```bash
pip install my-package
```

## Quick Start

```python
from my_package import Application
from my_package.models.data_models import InputData

app = Application()
data = [InputData(id="1", name="Sample", value=10.0, category="A")]
results = app.run(data)
```

## CLI Usage

```bash
# Process data file
my-package process -i input.json -o output.json

# Run migrations
my-package migrate --database-url sqlite:///data.db
```

## Configuration

Set environment variables or create a `.env` file:

```
DEBUG=false
DATABASE_URL=sqlite:///data.db
API_KEY=your-api-key
MAX_WORKERS=4
BATCH_SIZE=100
```

## Development

```bash
# Install development dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Run linting
flake8 src/ tests/
black src/ tests/

# Build documentation
cd docs && make html
```
```

### Phase 6: Deployment Preparation

#### Dockerfile
```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ src/
COPY pyproject.toml .

# Install package
RUN pip install -e .

# Create non-root user
RUN useradd --create-home --shell /bin/bash app
USER app

# Set default command
CMD ["my-package", "--help"]
```

#### Makefile
```makefile
.PHONY: install test lint format build clean deploy

install:
	pip install -e ".[dev]"

test:
	pytest tests/ -v --cov=src/my_package

lint:
	flake8 src/ tests/
	mypy src/

format:
	black src/ tests/
	isort src/ tests/

build:
	python -m build

clean:
	rm -rf build/ dist/ *.egg-info/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

deploy: clean build
	twine upload dist/*

docker-build:
	docker build -t my-package:latest .

docker-run:
	docker run --rm -it my-package:latest
```

## Best Practices Summary

1. **Separation of Concerns**: Each module has a single responsibility
2. **Dependency Injection**: Configuration and services are injected rather than hardcoded
3. **Error Handling**: Comprehensive exception handling with custom exceptions
4. **Type Safety**: Full type hints and Pydantic models for data validation
5. **Testing**: Comprehensive unit and integration tests
6. **Documentation**: Clear documentation for users and developers
7. **Configuration**: Environment-based configuration management
8. **Logging**: Structured logging throughout the application
9. **CLI Support**: User-friendly command-line interface
10. **Deployment Ready**: Docker support and proper packaging

This structure provides a solid foundation for transforming your 3500-line program into a maintainable, scalable, and deployment-ready Python package.