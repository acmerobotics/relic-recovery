import fieldImageName from '../assets/field.png';
const fieldImage = new Image();
fieldImage.src = fieldImageName;

const DEFAULT_OPTIONS = {
  padding: 15,
  gridLineColor: 'rgb(120, 120, 120)',
};

function scale(value, fromStart, fromEnd, toStart, toEnd) {
  return toStart + ((toEnd - toStart) * (value - fromStart) / (fromEnd - fromStart));
}

export default class Field {
  constructor(canvas, options) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.options = DEFAULT_OPTIONS;
    Object.assign(this.options, options || {});
    this.overlay = {
      ops: [],
    };
  }

  render(x, y, width, height) {
    this.canvas.width = this.canvas.width;
    const smallerDim = width < height ? width : height;
    const fieldSize = smallerDim - 2 * this.options.padding;
    this.renderField(
      x + (width - fieldSize) / 2,
      y + (height - fieldSize) / 2,
      fieldSize, fieldSize);
  }

  renderField(x, y, width, height) {
    this.ctx.save();
    this.ctx.globalAlpha = 0.25;
    this.ctx.drawImage(fieldImage, x, y, width, height);
    this.ctx.restore();
    this.renderGridLines(x, y, width, height, 7, 7);
    this.renderOverlay(x, y, width, height);
  }

  renderGridLines(x, y, width, height, numTicksX, numTicksY) {
    this.ctx.strokeStyle = this.options.gridLineColor;
    this.ctx.lineWidth = 1;

    const horSpacing = width / (numTicksX - 1);
    const vertSpacing = height / (numTicksY - 1);

    for (let i = 0; i < numTicksX; i += 1) {
      const lineX = x + horSpacing * i + 0.5;
      this.ctx.beginPath();
      this.ctx.moveTo(lineX, y);
      this.ctx.lineTo(lineX, y + height);
      this.ctx.stroke();
    }

    for (let i = 0; i < numTicksY; i += 1) {
      const lineY = y + vertSpacing * i + 0.5;
      this.ctx.beginPath();
      this.ctx.moveTo(x, lineY);
      this.ctx.lineTo(x + width, lineY);
      this.ctx.stroke();
    }
  }

  renderOverlay(x, y, width, height) {
    this.overlay.ops.forEach((op) => {
      switch (op.type) {
      case 'fill':
        this.ctx.fillStyle = op.color;
        break;
      case 'stroke':
        this.ctx.strokeStyle = op.color;
        break;
      case 'strokeWidth':
        this.ctx.lineWidth = op.width;
        break;
      case 'circle':
        this.ctx.beginPath();
        this.ctx.arc(
          scale(op.y, 72, -72, x, width + x),
          scale(op.x, 72, -72, y, height + y),
          op.radius, 0, 2 * Math.PI);
        if (op.stroke) {
          this.ctx.stroke();
        } else {
          this.ctx.fill();
        }
        break;
      case 'polygon': {
        this.ctx.beginPath();
        const { xPoints, yPoints, stroke } = op;
        this.ctx.moveTo(scale(yPoints[0], 72, -72, x, width + x),
          scale(xPoints[0], 72, -72, y, height + y));
        for (let i = 1; i < xPoints.length; i += 1) {
          this.ctx.lineTo(scale(yPoints[i], 72, -72, x, width + x),
            scale(xPoints[i], 72, -72, y, height + y));
        }
        this.ctx.closePath();
        if (stroke) {
          this.ctx.stroke();
        } else {
          this.ctx.fill();
        }
        break;
      }
      case 'polyline': {
        this.ctx.beginPath();
        const { xPoints, yPoints } = op;
        this.ctx.moveTo(scale(yPoints[0], 72, -72, x, width + x),
          scale(xPoints[0], 72, -72, y, height + y));
        for (let i = 1; i < xPoints.length; i += 1) {
          this.ctx.lineTo(scale(yPoints[i], 72, -72, x, width + x),
            scale(xPoints[i], 72, -72, y, height + y));
        }
        this.ctx.stroke();
        break;
      }
      default:
        console.log(`unknown op: ${op.type}`);
        console.log(op);
      }
    });
  }

  setOverlay(overlay) {
    this.overlay = overlay;
  }
}
