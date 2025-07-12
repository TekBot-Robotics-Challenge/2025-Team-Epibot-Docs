# API Documentation for Conveyor-Site

This document outlines the API endpoints available in the Conveyor-Site project, detailing the request methods, expected parameters, and response formats for each endpoint.

## API Endpoints

### Reset API

- **Endpoint:** `/api/reset`
- **Method:** `POST`
- **Description:** Resets the color counters and total count to zero.
- **Request Body:** None
- **Response:**
  - **Success Response:**
    - **Status Code:** 200 OK
    - **Body:**
      ```json
      {
        "success": true
      }
      ```
  - **Error Response:**
    - **Status Code:** 500 Internal Server Error
    - **Body:**
      ```json
      {
        "success": false
      }
      ```

### Data API

- **Endpoint:** `/api/data`
- **Method:** `GET`
- **Description:** Retrieves the current counts for each color and the total count.
- **Request Parameters:** None
- **Response:**
  - **Success Response:**
    - **Status Code:** 200 OK
    - **Body:**
      ```json
      {
        "RED": number,
        "GREEN": number,
        "BLUE": number,
        "YELLOW": number,
        "total": number
      }
      ```
  - **Error Response:**
    - **Status Code:** 500 Internal Server Error
    - **Body:**
      ```json
      {
        "error": "Erreur serveur"
      }
      ```

### Data Submission API

- **Endpoint:** `/api/data`
- **Method:** `POST`
- **Description:** Increments the count for a specified color and the total count.
- **Request Body:**
  - **Example:**
    ```json
    {
      "color": "RED"
    }
    ```
- **Response:**
  - **Success Response:**
    - **Status Code:** 200 OK
    - **Body:**
      ```json
      {
        "color": "RED"
      }
      ```
  - **Error Response:**
    - **Status Code:** 400 Bad Request
    - **Body:**
      ```json
      {
        "error": "Couleur invalide"
      }
      ```
    - **Status Code:** 500 Internal Server Error
    - **Body:**
      ```json
      {
        "error": "Erreur serveur"
      }
      ```

## Notes

- Ensure that the request body for the POST methods is correctly formatted as JSON.
- The API is designed to handle errors gracefully, returning appropriate status codes and messages for different failure scenarios.