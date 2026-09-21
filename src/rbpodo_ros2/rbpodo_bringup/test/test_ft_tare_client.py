# Copyright (c) 2024 Rainbow Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the F/T tare helper's two forwarding paths."""

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from rbpodo_bringup.ft_tare_client import FtTareClient
from std_srvs.srv import Trigger


class _DoneFuture:

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _RecordingClient:

    def __init__(self, success=True, message="ok"):
        self.requests = []
        self._result = SimpleNamespace(success=success, message=message)

    def call_async(self, request):
        self.requests.append(request)
        return _DoneFuture(self._result)


def _make_uninitialized_node():
    node = FtTareClient.__new__(FtTareClient)
    node._lock = threading.Lock()
    node._tare_client = _RecordingClient(message="strict")
    node._confirmed_tare_client = _RecordingClient(message="supervised")
    node._runtime_tare_client = _RecordingClient(message="runtime")
    return node


def test_strict_tare_keeps_using_strict_hardware_service_client():
    node = _make_uninitialized_node()
    logger = MagicMock()

    with patch.object(FtTareClient, "get_logger", return_value=logger):
        result = node.tare(timeout_sec=1.0)

    assert result == (True, "strict")
    assert len(node._tare_client.requests) == 1
    assert isinstance(node._tare_client.requests[0], Trigger.Request)
    assert node._confirmed_tare_client.requests == []
    assert node._runtime_tare_client.requests == []


def test_confirmed_tare_uses_supervised_client_and_emits_safety_warning():
    node = _make_uninitialized_node()
    logger = MagicMock()

    with patch.object(FtTareClient, "get_logger", return_value=logger):
        result = node.confirm_free_space_and_tare(timeout_sec=1.0)

    assert result == (True, "supervised")
    assert node._tare_client.requests == []
    assert len(node._confirmed_tare_client.requests) == 1
    warning = logger.warning.call_args.args[0]
    assert "stationary" in warning
    assert "roller is fully free" in warning


def test_hardware_service_names_are_explicit_and_distinct():
    assert FtTareClient.HARDWARE_TARE_SERVICE == "/rbpodo_ft_tare/tare_ft"
    assert FtTareClient.HARDWARE_CONFIRMED_TARE_SERVICE == (
        "/rbpodo_ft_tare/confirm_free_space_and_tare"
    )
    assert FtTareClient.HARDWARE_RUNTIME_TARE_SERVICE == (
        "/rbpodo_ft_tare/runtime_free_space_tare"
    )


def test_runtime_tare_uses_runtime_client_and_emits_precontact_warning():
    node = _make_uninitialized_node()
    logger = MagicMock()

    with patch.object(FtTareClient, "get_logger", return_value=logger):
        result = node.runtime_free_space_tare(timeout_sec=1.0)

    assert result == (True, "runtime")
    assert len(node._runtime_tare_client.requests) == 1
    assert isinstance(node._runtime_tare_client.requests[0], Trigger.Request)
    assert node._tare_client.requests == []
    assert node._confirmed_tare_client.requests == []
    warning = logger.warning.call_args.args[0]
    assert "pre-contact" in warning
    assert "fully free" in warning
