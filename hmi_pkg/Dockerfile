FROM osrf/ros:humble-desktop

WORKDIR /app

# Install Python and system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better layer caching
COPY requirements.txt .

# Install Python packages (excluding rclpy which comes from ROS)
RUN pip install --no-cache-dir \
    fastapi==0.104.1 \
    uvicorn[standard]==0.24.0 \
    python-multipart==0.0.6 \
    websockets==12.0 \
    openai>=1.12.0

# Copy application files from hmi_pkg directory structure
COPY nodes/ /app/hmi_pkg/nodes/
COPY config/ /app/hmi_pkg/config/
COPY public/ /app/hmi_pkg/public/

# Create __init__.py files for Python package
RUN touch /app/hmi_pkg/__init__.py
RUN touch /app/hmi_pkg/nodes/__init__.py

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose port
EXPOSE 8000

# Set environment variables
ENV HMI_ROS_ENABLE=1
ENV PYTHONPATH=/app/hmi_pkg/nodes:$PYTHONPATH

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:8000/status || exit 1

# Run the application with ROS environment
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /app/hmi_pkg/nodes/hmi_server.py"]
