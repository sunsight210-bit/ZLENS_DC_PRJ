// Test/test_storage_task.cpp
#include <gtest/gtest.h>
#include "storage_task.hpp"
#include "crc16.hpp"

using namespace zlens;

// Global required by task_config.hpp
namespace zlens {
    volatile bool g_bSpiEmergency = false;
}

class StorageTaskTest : public ::testing::Test {
protected:
    StorageTask task;
    FramStorage fram;
    SPI_HandleTypeDef hspi;
    QueueHandle_t saveQ;

    void SetUp() override {
        mock::get_log().reset();
        mock::get_log().tick_auto_increment = false;
        mock::get_log().tick = 0;
        g_bSpiEmergency = false;

        hspi.Instance = SPI2;
        fram.init(&hspi);

        saveQ = xQueueCreate(8, sizeof(SAVE_MESSAGE_S));
        task.init(&fram, saveQ);
    }

    void send_save(int32_t pos, uint16_t zoom, uint8_t reason) {
        SAVE_MESSAGE_S msg = {pos, zoom, reason, 0, 0};
        xQueueSend(saveQ, &msg, 0);
    }
};

TEST_F(StorageTaskTest, ReceiveSave_Arrived_CallsFram) {
    send_save(10000, 60, save_reason::ARRIVED);
    task.run_once();

    EXPECT_GT(mock::get_log().spi_tx_data.size(), 0u);
    EXPECT_EQ(task.get_state().current_position, 10000);
    EXPECT_EQ(task.get_state().current_zoom_x10, 60);
}

TEST_F(StorageTaskTest, ReceiveSave_Stall_CallsFram) {
    send_save(5000, 40, save_reason::STALL);
    task.run_once();

    EXPECT_GT(mock::get_log().spi_tx_data.size(), 0u);
    EXPECT_EQ(task.get_state().last_save_reason, save_reason::STALL);
}

TEST_F(StorageTaskTest, PeriodicSave_After500ms) {
    send_save(1000, 60, save_reason::ARRIVED);
    task.run_once();
    size_t spi_count_after_first = mock::get_log().spi_tx_data.size();

    // Update position without queue message
    FRAM_STATE_S st = task.get_state();
    st.current_position = 2000;
    task.set_state(st);

    mock::get_log().tick = 501;
    task.run_once();

    EXPECT_GT(mock::get_log().spi_tx_data.size(), spi_count_after_first);
}

TEST_F(StorageTaskTest, SpiEmergency_SkipsSave) {
    g_bSpiEmergency = true;
    send_save(10000, 60, save_reason::ARRIVED);

    size_t spi_count_before = mock::get_log().spi_tx_data.size();
    task.run_once();

    EXPECT_EQ(mock::get_log().spi_tx_data.size(), spi_count_before);
}

TEST_F(StorageTaskTest, RestoreState_Invalid) {
    FRAM_STATE_S restored;
    bool ok = task.restore_state(restored);
    EXPECT_FALSE(ok);
}

TEST_F(StorageTaskTest, UpdatesPosition_InState) {
    send_save(1000, 60, save_reason::ARRIVED);
    task.run_once();
    EXPECT_EQ(task.get_state().current_position, 1000);

    send_save(2000, 80, save_reason::ARRIVED);
    task.run_once();
    EXPECT_EQ(task.get_state().current_position, 2000);
    EXPECT_EQ(task.get_state().current_zoom_x10, 80);
}

TEST_F(StorageTaskTest, HomingDone_ResetsMovCount) {
    // homing_done=1 resets move_count to 0, then position_valid=0xFF increments to 1
    SAVE_MESSAGE_S msg = {10000, 60, save_reason::ARRIVED, 1, 0xFF};
    xQueueSend(saveQ, &msg, 0);
    task.run_once();

    EXPECT_EQ(task.get_state().homing_done, 1);
    EXPECT_EQ(task.get_state().position_valid, 0xFF);
    EXPECT_EQ(task.get_state().move_count, 1);  // reset to 0, then +1 from position_valid
}

TEST_F(StorageTaskTest, MultipleSaves_Sequential) {
    send_save(1000, 60, save_reason::ARRIVED);
    send_save(2000, 80, save_reason::ARRIVED);
    send_save(3000, 100, save_reason::ARRIVED);

    task.run_once();
    EXPECT_EQ(task.get_state().current_position, 1000);
    task.run_once();
    EXPECT_EQ(task.get_state().current_position, 2000);
    task.run_once();
    EXPECT_EQ(task.get_state().current_position, 3000);
}
