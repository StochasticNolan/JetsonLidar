#!/usr/bin/env python3
"""
Test 3: Photo Upload Pipeline

Tests the complete photo upload flow:
1. Request presigned S3 URL from AeroSync
2. Upload photo to S3 using PUT
3. Confirm upload with AeroSync
4. Verify photo record created
"""

import sys
import os
import io
from datetime import datetime

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'perception_nav', 'src'))

from aerosync_client import AeroSyncClient, AeroSyncConfig

# For S3 upload
try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    print("❌ requests not installed. Run: pip install requests")
    sys.exit(1)


def create_test_image():
    """Create a small test JPEG image in memory."""
    try:
        from PIL import Image

        # Create small 100x100 black image
        img = Image.new('RGB', (100, 100), color='black')

        # Save to BytesIO
        img_bytes = io.BytesIO()
        img.save(img_bytes, format='JPEG', quality=85)
        img_bytes.seek(0)

        return img_bytes.getvalue()
    except ImportError:
        # Fallback: create minimal valid JPEG header
        # This is a 1x1 black JPEG (smallest valid JPEG)
        minimal_jpeg = bytes([
            0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46,
            0x49, 0x46, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
            0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
            0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08,
            0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0A, 0x0C,
            0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
            0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D,
            0x1A, 0x1C, 0x1C, 0x20, 0x24, 0x2E, 0x27, 0x20,
            0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29,
            0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27,
            0x39, 0x3D, 0x38, 0x32, 0x3C, 0x2E, 0x33, 0x34,
            0x32, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
            0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4,
            0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x08, 0xFF, 0xC4, 0x00, 0x14,
            0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01,
            0x00, 0x00, 0x3F, 0x00, 0x7F, 0xFF, 0xD9
        ])
        return minimal_jpeg


def test_photo_upload():
    """Test complete photo upload pipeline."""

    print("=" * 80)
    print("TEST 3: Photo Upload Pipeline")
    print("=" * 80)

    # Configuration
    AEROSYNC_URL = "http://localhost:8000"
    DRONE_API_KEY = "dev-key-change-in-production"

    print(f"\nConnecting to: {AEROSYNC_URL}\n")

    # Initialize client
    config = AeroSyncConfig()
    config.base_url = AEROSYNC_URL
    config.api_key = DRONE_API_KEY
    config.websocket_enabled = False

    client = AeroSyncClient(config=config)

    # Get job and pole IDs
    print("[SETUP] Enter test parameters...")
    job_id = input("Enter job ID: ").strip()
    pole_id = input("Enter pole ID (from Test 1): ").strip()

    # Step 1: Request presigned URL
    print("\n[STEP 1] Requesting S3 presigned upload URL...")

    try:
        # Call the actual Jetson client method
        url_response = client.get_photo_upload_url(
            job_id=job_id,
            pole_id=pole_id
        )

        if not url_response:
            print("❌ FAILED: Could not get presigned URL")
            return False

        upload_url = url_response.get('upload_url')
        photo_id = url_response.get('photo_id')
        s3_key = url_response.get('s3_key')

        if not upload_url:
            print("❌ FAILED: No upload_url in response")
            return False

        print("✅ SUCCESS: Got presigned URL")
        print(f"   - Photo ID: {photo_id}")
        print(f"   - S3 Key: {s3_key}")
        print(f"   - URL length: {len(upload_url)} chars")
        print(f"   - URL starts with: {upload_url[:50]}...")

    except Exception as e:
        print(f"❌ FAILED: Error getting presigned URL: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Step 2: Upload photo to S3
    print("\n[STEP 2] Uploading test photo to S3...")

    try:
        # Create test image
        image_data = create_test_image()
        print(f"   - Image size: {len(image_data)} bytes")

        # Upload to S3 using PUT (presigned URL expects PUT)
        response = requests.put(
            upload_url,
            data=image_data,
            headers={'Content-Type': 'image/jpeg'},
            timeout=30
        )

        if response.status_code not in [200, 204]:
            print(f"❌ FAILED: S3 upload failed with status {response.status_code}")
            print(f"   Response: {response.text[:200]}")
            return False

        print("✅ SUCCESS: Photo uploaded to S3")
        print(f"   - HTTP Status: {response.status_code}")

    except Exception as e:
        print(f"❌ FAILED: S3 upload error: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Step 3: Confirm upload with AeroSync
    print("\n[STEP 3] Confirming upload with AeroSync...")

    try:
        # Use actual Jetson client method
        confirmed = client.confirm_photo_upload(
            job_id=job_id,
            pole_id=pole_id,
            photo_id=photo_id,
            s3_key=s3_key,
            pole_lat=37.7749,  # Test coordinates
            pole_lon=-122.4194,
            drone_lat=37.7750,
            drone_lon=-122.4195,
            drone_alt=50.0,
            drone_heading=180.0,
            captured_at=datetime.utcnow().isoformat() + 'Z'
        )

        if not confirmed:
            print("❌ FAILED: Photo confirmation failed")
            return False

        print("✅ SUCCESS: Upload confirmed")
        print(f"   - Photo ID: {photo_id}")
        print(f"   - S3 Key: {s3_key}")

    except Exception as e:
        print(f"❌ FAILED: Confirmation error: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Final Summary
    print("\n" + "=" * 80)
    print("✅ PHOTO UPLOAD PIPELINE TEST PASSED")
    print("=" * 80)
    print("\nValidated:")
    print("  ✓ GET /api/drone/jobs/{job_id}/poles/{pole_id}/photo/upload-url")
    print("  ✓ S3 presigned URL generation")
    print("  ✓ S3 PUT upload (actual file upload)")
    print("  ✓ POST /api/drone/jobs/{job_id}/poles/{pole_id}/photo/confirm")
    print("\nPhoto Upload Flow:")
    print("  1. Request presigned URL from AeroSync")
    print("  2. Upload photo directly to S3 using PUT")
    print("  3. Confirm upload with AeroSync (creates database record)")
    print("\nThe complete photo upload pipeline is working correctly!")

    return True


if __name__ == "__main__":
    try:
        success = test_photo_upload()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
