/*******************************************************************************
 * Copyright (C) Lawrence Lo (https://github.com/galliumstudio). 
 * All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hsm_id.h"
#include "fw_log.h"
#include "MoistureSensor.h"
#include "event.h"
#include "bsp.h"

//Q_DEFINE_THIS_FILE

namespace APP {

ADC_HandleTypeDef g_AdcHandle;



void MoistureSensor::ConfigADC() {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    // Using analog pin 0 PA_0
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
 
    ADC_ChannelConfTypeDef adcChannel;
 
    g_AdcHandle.Instance = ADC1;
 
    g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = DISABLE;
    g_AdcHandle.Init.ContinuousConvMode = ENABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 0;
    g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 1;
    g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
    g_AdcHandle.Init.EOCSelection = DISABLE;
 
    HAL_ADC_Init(&g_AdcHandle);
 
    adcChannel.Channel = ADC_CHANNEL_11;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcChannel.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
        asm("bkpt 255");
    }
}

void MoistureSensor::ReadADC() {
    // Retreives the current value from the moisture sensore by reading the ADC
    uint32_t g_ADCValue;
    HAL_ADC_Start(&g_AdcHandle);
    if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK) {
        g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
    }
    PRINT("Moisture Sensor Value = %d\r\n", g_ADCValue);
}


    
MoistureSensor::MoistureSensor() :
    QActive((QStateHandler)&MoistureSensor::InitialPseudoState), 
    m_id(MOISTURE_SENSOR), m_name("MOISTURE_SENSOR"), m_nextSequence(0),
    m_waitTimer(this, MOISTURE_SENSOR_WAIT_TIMER) {}

QState MoistureSensor::InitialPseudoState(MoistureSensor * const me, QEvt const * const e) {
    (void)e;
    
    me->subscribe(MOISTURE_SENSOR_START_REQ);
    me->subscribe(MOISTURE_SENSOR_STOP_REQ);
    me->subscribe(MOISTURE_SENSOR_WAIT_TIMER);
    me->subscribe(MOISTURE_SENSOR_STATE_TIMER);

    return Q_TRAN(&MoistureSensor::Root);
}

QState MoistureSensor::Root(MoistureSensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            ConfigADC();
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_TRAN(&MoistureSensor::Stopped);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm::top);
            break;
        }
    }
    return status;
}

QState MoistureSensor::Stopped(MoistureSensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            me->m_waitTimer.armX(10000); // 10s timer
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case MOISTURE_SENSOR_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new MoistureSensorStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case MOISTURE_SENSOR_WAIT_TIMER: {
            LOG_EVENT(e);
            status = Q_TRAN(&MoistureSensor::Started);
            break;
        }
         case MOISTURE_SENSOR_START_REQ: {
            LOG_EVENT(e);
            me->m_waitTimer.disarm();
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new MoistureSensorStartCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_TRAN(&MoistureSensor::Started);
            break;
        }
        default: {
            status = Q_SUPER(&MoistureSensor::Root);
            break;
        }
    }
    return status;
}

QState MoistureSensor::Started(MoistureSensor * const me, QEvt const * const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            ReadADC();
            Evt *evt = new MoistureSensorStopReq(me->m_nextSequence++);
            QF::PUBLISH(evt, me);
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status = Q_HANDLED();
            break;
        }
        case Q_INIT_SIG: {
            status = Q_HANDLED();
            break;
        }
        case MOISTURE_SENSOR_STOP_REQ: {
            LOG_EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new MoistureSensorStopCfm(req.GetSeq(), ERROR_SUCCESS);
            QF::PUBLISH(evt, me);
            status = Q_TRAN(&MoistureSensor::Stopped);
            break;
        }
        default: {
            status = Q_SUPER(&MoistureSensor::Root);
            break;
        }
    }
    return status;
}


} // namespace APP
