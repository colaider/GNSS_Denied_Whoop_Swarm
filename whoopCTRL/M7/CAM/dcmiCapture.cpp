// dcmi:: on the board (see frameCapture.h): a DCMI snapshot into any buffer, through the DMA
// stream the camera library set up (DMA2 Stream 3: request DCMI, peripheral -> memory, words).
// The library's grabFrame() must have run once before (CameraReader::begin does it): that
// configures the stream, the DCMI crop window and the interrupts. Here only the buffer address,
// the length and the enable bits are touched, as HAL_DCMI_Start_DMA / HAL_DCMI_Stop do, but
// without waiting.

#include "headers/frameCapture.h"

#include "stm32h7xx_hal.h"

#include "../../shared/headers/sharedClock.h"

// 'dcmi' prefix: sketches include several .cpp files into one compile unit
static volatile uint32_t dcmiEnds = 0, dcmiEndUs = 0;

// The camera library's DCMI_IRQHandler runs the HAL, which calls this at the end of every
// captured frame (weak in the HAL, not used by the library): the exact end of the readout.
extern "C" void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef*) {
  dcmiEndUs = shared::nowUs();
  dcmiEnds = dcmiEnds + 1;
}

namespace vision {
namespace dcmi {

void start(uint8_t* buf, uint32_t bytes) {
  DMA_Stream_TypeDef* s = DMA2_Stream3;
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {}
  DMA2->LIFCR = DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;
  // polled, so the library's DMA interrupt stays quiet (its handle thinks nothing runs)
  s->CR  &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_DBM);
  s->FCR &= ~DMA_SxFCR_FEIE;
  s->PAR  = (uint32_t)&DCMI->DR;
  s->M0AR = (uint32_t)buf;
  s->NDTR = bytes / 4;                         // one frame (75 KB = 19200 words < 65535: no double-buffer mode)

  // no dirty cache lines over buf may be written back over the DMA's data later
  SCB_InvalidateDCache_by_Addr((uint32_t*)buf, (int32_t)bytes);

  DCMI->ICR = DCMI_ICR_FRAME_ISC | DCMI_ICR_OVR_ISC | DCMI_ICR_ERR_ISC | DCMI_ICR_VSYNC_ISC | DCMI_ICR_LINE_ISC;
  DCMI->CR |= DCMI_CR_CM;                      // snapshot: one frame, then CAPTURE clears itself
  DCMI->CR |= DCMI_CR_ENABLE;
  s->CR |= DMA_SxCR_EN;
  // only the frame-end interrupt (-> HAL_DCMI_FrameEventCallback; the HAL clears it again).
  // Not error / overrun: the HAL would abort the library's DMA handle, which is not running.
  DCMI->IER = DCMI_IER_FRAME_IE;
  DCMI->CR |= DCMI_CR_CAPTURE;                 // starts at the next frame's VSYNC
}

bool busy() { return (DCMI->CR & DCMI_CR_CAPTURE) != 0; }

void stop(uint8_t* buf, uint32_t bytes) {
  DCMI->CR &= ~DCMI_CR_CAPTURE;                // (already clear after a complete frame)
  DCMI->CR &= ~DCMI_CR_ENABLE;
  DMA_Stream_TypeDef* s = DMA2_Stream3;
  s->CR &= ~DMA_SxCR_EN;
  while (s->CR & DMA_SxCR_EN) {}
  SCB_InvalidateDCache_by_Addr((uint32_t*)buf, (int32_t)bytes);   // read the DMA's data, not old cache lines
}

uint32_t frameEnds() { return dcmiEnds; }
uint32_t frameEndUs() { return dcmiEndUs; }

}  // namespace dcmi
}  // namespace vision
