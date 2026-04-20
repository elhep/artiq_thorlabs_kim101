# Base Python image
FROM python:3.10

# Set the working directory inside the container
WORKDIR /usr/src/app

# Copy application files into the container
COPY . .

WORKDIR /usr/src/app

RUN apt-get update

# Install artiq_iseg_hv_psu module
RUN pip install .

ENV PYTHONUNBUFFERED=1

# Specify the default command to run the service
# CMD ["python", "kim101_driver/aqctl_kim101_driver.py", "--simulation"]
CMD ["pytest", "kim101_driver/test_kim101_driver.py"]