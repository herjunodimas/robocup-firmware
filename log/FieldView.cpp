#include "FieldView.hpp"

#include <Constants.hpp>
#include <QPainter>
#include <QResizeEvent>

#include "drawing/Elements.hpp"

using namespace Constants;
using namespace Log;

FieldView::FieldView(QWidget* parent) :
	QGLWidget(parent), _team(UnknownTeam)
{
	_frame = 0;

	_tx = -Field::Length/2.0f;
	_ty = 0;
	_ta = -90;
}

void FieldView::team(Team t)
{
    _team = t;

    if (_team == Blue)
    {
        _tx = Field::Length/2.0f;
        _ta = 90;
    }
}

void FieldView::frame(Packet::LogFrame* frame)
{
    _frame = frame;
    update();
}

void FieldView::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	//painter.setRenderHint(QPainter::SmoothPixmapTransform);
	painter.scale(width()/Floor::Length, -height()/Floor::Width);

	// world space
	painter.translate(Floor::Length/2.0, -Floor::Width/2.0);

	drawField(painter);

	////team space
	painter.translate(_tx, _ty);
	painter.rotate(_ta);
    
	if (_frame)
	{
		//TODO handle display options...
		
		//draw the raw frame info
		//TODO draw only the last frame?? - Roman
		Q_FOREACH(const Packet::Vision& vis, _frame->rawVision)
		{
			if (vis.sync)
			{
				continue;
			}
			
			Q_FOREACH(const Packet::Vision::Robot& r, vis.blue)
			{
				drawRobot(painter, Blue, r.shell, r.pos, r.angle);
			}
			
			Q_FOREACH(const Packet::Vision::Robot& r, vis.yellow)
			{
				drawRobot(painter, Yellow, r.shell, r.pos, r.angle);
			}
			
			Q_FOREACH(const Packet::Vision::Ball& b, vis.balls)
			{
				drawBall(painter, b.pos);
			}
		}
		
		/*
		for (unsigned int i=0 ; i<5 ; ++i)
		{
			const Packet::LogFrame::Robot& s = _frame->self[i];
			const Packet::LogFrame::Robot& o = _frame->opp[i];

			if (s.valid)
			{
				drawRobot(painter, _team, s.shell, s.pos, s.angle);
			}

			if (o.valid)
			{
				drawRobot(painter, opponentTeam(_team), o.shell, o.pos, o.angle);
			}
		}

		if (_frame->ball.valid)
		{
			drawBall(painter, _frame->ball.pos);
		}
		*/
	}
}

void FieldView::resizeEvent(QResizeEvent* event)
{
	int w = event->size().width();
	int h = int(w * Constants::Floor::Aspect);

	if (h > event->size().height())
	{
		h = event->size().height();
		w = int(h/Constants::Floor::Aspect);
	}

	this->resize(w,h);
	event->accept();
}
