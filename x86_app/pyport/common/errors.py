class CliError(Exception):
    """Custom error class for command line interface errors."""
    def __init__(self, message):
        super().__init__(message)
        self.message = message

    def __str__(self):
        return f"CliError: {self.message}"
